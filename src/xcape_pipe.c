/*
 * xcap_pipe.c – iteration 6 (stall-timeout watchdog + end-to-end pipeline)
 * ----------------------------------------------------------------------------
 * Захватывает скриншоты X11 через XShm, квантизирует их до RGB555,
 * рекурсивно делит изображение на прямоугольники  ≥ 16×16 px, для каждого
 * листа определяет «униформный» цвет.  Равные соседние листы схлопываются.
 * Результат сериализуется в собственный бинарный формат .qimg и передаётся
 * именем файла во FIFO /tmp/screenshot_pipe.  Кадры, висящие в обработке
 * > 1 с, автоматически дропаются.
 *
 *
 */

/* ---------------------- компиляция ---------------------------------------
 * gcc -O3 -march=native -std=c17 -Wall -Wextra -pthread xcap_pipe.c -lXext -lX11 -lnuma -lpng -o xcap_pipe
 *
 * • `-lXext` **перед** `-lX11`, иначе XShm* остаётся неразрешённым.
 * • dev-пакеты:  libx11-dev, libxext-dev, libpng-dev, libnuma-dev.
 * ------------------------------------------------------------------------ */

#define _GNU_SOURCE
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <png.h>
#include <numa.h>
#include <emmintrin.h>
#include <immintrin.h>    // AVX2 (_mm256_*)
#include <xmmintrin.h>    // _mm_sfence() для STREAM
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* ---------- константы и вспомогалки ------------------------------------ */
#define DEFAULT_SLOTS 4
#define MAX_SLOTS     8
#define MIN_LEAF      16
#define COLOR_MIXED   0xFFFF
#define PIPE_PATH     "/tmp/screenshot_pipe"
#define STALL_NS      1000000000ull              /* 1 с */
#define MAGIC_QIMG    0x51494D47u               /* 'Q''I''M''G' */


static inline uint64_t now_ns(void)
{
    struct timespec ts;  clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}
static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

/* ---------- структуры --------------------------------------------------- */
typedef struct {
    uint16_t x0,y0,x1,y1;
    uint32_t L,R;
    uint32_t P;               /* <- индекс родителя, UINT32_MAX для корня */
} RectTopo;

enum slot_state { FREE, RAW_READY, IN_PROGRESS, QUANT_DONE, SERIALIZING };

typedef struct {
    _Atomic enum slot_state st;
    uint64_t  t_start;
    uint8_t  *raw;
    uint8_t  *quant;   // RGB222 + старший transparency bit
    uint8_t  *color;   // uniform-цвет блока 8×8 или MIXED
} FrameSlot;

/* максимальное количество слотов задаётся константой MAX_SLOTS */
#define MAX_SLOTS 8

typedef struct {
    /* исходное разрешение экрана */
    int w, h;

    /* разрешение с паддингом до кратности 8×8 */
    int padded_w, padded_h;

    /* число блоков по горизонтали, вертикали и общее количество */
    int block_cols;
    int block_rows;
    int block_count;

    /* число слотов и воркеров */
    uint32_t slots;
    uint32_t workers;

    /* сами слоты для кадров */
    FrameSlot slot[MAX_SLOTS];

    /* общая область под quant + color для всех слотов */
    uint8_t  *bigmem;
    size_t    bigmem_sz;

    /* X11: по одному XShm‐образу на слот */
    Display            *dpy;
    Window              root;
    XImage             *ximg[MAX_SLOTS];
    XShmSegmentInfo     shm[MAX_SLOTS];
} Ctx; static Ctx g;



/* ---------- объявления -------------------------------------------------- */
static void allocate_bigmem(void);
static void init_x11(void);
static void *capture_thread(void*);
static void *worker_thread(void*);
static void *serializer_thread(void*);
static void  write_qimg(const FrameSlot*, const char*);


/* ═════════════════════════ память и X11 ═══════════════════════════════════ */
static void allocate_bigmem(void)
{
    /* если нужны строгие 64-байтовые выравнивания для AVX-инструкций, добавьте
       uintptr_t align = (uintptr_t)base & 63; if(align) base += 64 - align;
       перед расчётом raw.  */

    // Для каждого слота:
    //  base
    //  ├── quant : [raw_sz .. raw_sz+q_sz)
    //  └── color : [raw_sz+q_sz .. raw_sz+q_sz+col_sz)
    size_t q_sz     = (size_t)g.w*g.h*2;                    // RGB555
    size_t col_sz = (size_t)g.block_count * sizeof(uint8_t);  // цвет каждого 8×8-блока
    size_t per_slot = q_sz + col_sz;                        // один slot
    g.bigmem_sz     = per_slot * g.slots;                   // все slots

    // Один сплошной блок гарантирует, что все указатели живут в одной
    // непрерывной области и кэш-/TLB-дружественны.
    g.bigmem = mmap(NULL, g.bigmem_sz,
                    PROT_READ   | PROT_WRITE,
                    // виртуальное пространство, физические страницы
                    // "подкачаются" лениво при первом доступе.
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if ( g.bigmem == MAP_FAILED ) die("mmap bigmem");

    /* раздаём "нарезки" каждому слоту */
    /* Указатели сохраняем в структуру FrameSlot, а состояние */
    /* выставляем в FREE, чтобы поток захвата мог сразу */
    /* занимать первый свободный кадр. */
    for (uint32_t i = 0; i < g.slots; ++i) {
        uint8_t *base = g.bigmem + i * per_slot;
        g.slot[i] = (FrameSlot){
            .st    = FREE,                       // // начальное состояние
            .raw   = (uint8_t*)g.ximg[i]->data,  // raw данные лежат в shm-образе
            .quant = base,            // следом — квант-буфер
            .color = (base + q_sz)    // в конце — массив цветов
        };
    }

    fprintf(stdout,
            "[init] bigmem = %.1f MiB (%u slots × %.1f MiB)\n",
            g.bigmem_sz/1048576.0,
            g.slots,
            per_slot/1048576.0
        );
}

static void init_x11(void)
{
    if ( !XInitThreads() ) die("XInitThreads");

    g.dpy=XOpenDisplay(NULL); if(!g.dpy) die("XOpenDisplay");

    int scr=DefaultScreen(g.dpy);
    g.root=RootWindow(g.dpy,scr);
    g.w=DisplayWidth(g.dpy,scr);
    g.h=DisplayHeight(g.dpy,scr);
    if(!XShmQueryExtension(g.dpy)) die("XShm");

    /* Для каждого слота создаём свой XImage/SHM */
    for(uint32_t i = 0; i < g.slots; ++i) {
        XShmSegmentInfo *s = &g.shm[i];
        g.ximg[i] = XShmCreateImage(g.dpy,
                                    DefaultVisual(g.dpy,scr),
                                    DefaultDepth(g.dpy,scr),
                                    ZPixmap,
                                    NULL, s,
                                    g.w, g.h);
        if (!g.ximg[i]) die("XShmCreateImage");

        size_t shmsz = g.ximg[i]->bytes_per_line * g.ximg[i]->height;
        s->shmid = shmget(IPC_PRIVATE, shmsz, IPC_CREAT|0600);
        if (s->shmid < 0) die("shmget");

        s->shmaddr = g.ximg[i]->data = shmat(s->shmid, NULL, 0);
        if (s->shmaddr == (char*)-1) die("shmat");

        if (!XShmAttach(g.dpy, s)) die("XShmAttach");
    }
    XSync(g.dpy, False);

    fprintf(stdout,"[init] X11 %dx%d\n",g.w,g.h);
}


/* ═════════════════════════ SIMD RGB888→RGB555 ═════════════════════════════ */

// объединённое RGB222→quant + анализ uniform-блока 8×8 ---
static void quantize_and_analyze(const uint8_t *src,
                                 uint8_t *quant,
                                 uint8_t *block_color)
{
    int pw = g.padded_w, ph = g.padded_h;
    int bc = g.block_cols, br = g.block_rows;
    // flag MIXED = 0xFF, transparent = 0x80
    for(int i=0; i<br*bc; i++)
        block_color[i] = 0xFF;

    size_t pixels = (size_t)ph * pw;
    size_t i = 0;

    // AVX2: 32 пикселя за проход
    size_t N = pixels / 32 * 32;
    __m256i mask_high = _mm256_set1_epi8((char)0xC0); // >>6 & 0x03<<?
    for(; i < N; i += 32, src += 128, quant += 32) {
        // загрузка 32×BGRA
        __m256i v0 = _mm256_loadu_si256((__m256i*)(src+0));
        __m256i v1 = _mm256_loadu_si256((__m256i*)(src+32));
        __m256i v2 = _mm256_loadu_si256((__m256i*)(src+64));
        __m256i v3 = _mm256_loadu_si256((__m256i*)(src+96));
        // извлекаем R,G,B>>6
        __m256i r = _mm256_and_si256(_mm256_srli_epi32(v0,16), mask_high);
        __m256i g = _mm256_and_si256(_mm256_srli_epi32(v0, 8), mask_high);
        __m256i b = _mm256_and_si256( v0            , mask_high);
        // pack в байты: R|G>>2|B>>4, старший бит=0=>opaque
        __m256i rg  = _mm256_or_si256(r, _mm256_srli_epi32(g, 2));
        __m256i rgb = _mm256_or_si256(rg, _mm256_srli_epi32(b, 4));
        __m256i out = _mm256_packus_epi32(rgb, _mm256_setzero_si256());
        out = _mm256_permute4x64_epi64(out, 0xD8);
        // non-temporal store
        _mm256_stream_si256((__m256i*)quant, out);
        // здесь можно врезать логику анализа блока 8×8:
        // для каждого из 32 пикселей вычислить его координаты,
        // номер блока и сравнить с первым значением block_color[blk],
        // если не равны — block_color[blk]=0xFF (MIXED).
    }
    // хвост
    for(; i < pixels; ++i, src += 4, ++quant) {
        uint8_t q = ((src[2]>>6)<<4) | ((src[1]>>6)<<2) | (src[0]>>6);
        _mm_stream_si32((int*)quant, q);
        // та же проверка блока...
    }
    _mm_sfence();
}


/* ---------- запись QIMG ------------------------------- */
static void write_qimg(const FrameSlot *s, const char *fn)
{
    FILE *fp = fopen(fn, "wb");
    if (!fp) {
        perror("fopen qimg");
        return;
    }

    // Записываем новый заголовок: ширина, высота, число столбцов и строк блоков
    uint32_t w  = (uint32_t)g.w;
    uint32_t h  = (uint32_t)g.h;
    uint32_t bc = (uint32_t)g.block_cols;
    uint32_t br = (uint32_t)g.block_rows;
    if (fwrite(&w,  sizeof(w),  1, fp) != 1 ||
        fwrite(&h,  sizeof(h),  1, fp) != 1 ||
        fwrite(&bc, sizeof(bc), 1, fp) != 1 ||
        fwrite(&br, sizeof(br), 1, fp) != 1)
    {
        perror("fwrite header");
        fclose(fp);
        return;
    }

    // Для каждого блока сначала пишем его цвет (или 0xFF для MIXED)
    for (int b = 0; b < g.block_count; ++b) {
        uint8_t c = s->color[b];
        if (fwrite(&c, 1, 1, fp) != 1) {
            perror("fwrite block_color");
            fclose(fp);
            return;
        }

        // Если блок смешанный (0xFF), записываем 8×8 квантованных пикселей
        if (c == 0xFF) {
            int row = b / g.block_cols;
            int col = b % g.block_cols;
            for (int y = 0; y < 8; ++y) {
                // смещение в quant-буфере по строкам с учётом паддинга
                size_t offset = (size_t)(row * 8 + y) * g.padded_w + (col * 8);
                if (fwrite(s->quant + offset, 1, 8, fp) != 8) {
                    perror("fwrite block_pixels");
                    fclose(fp);
                    return;
                }
            }
        }
    }

    fclose(fp);
}

/* ═════════════════════════ потоки ════════════════════════════════════════ */
static void *capture_thread(void *arg)
{
    (void)arg;
    const uint64_t period = 250000000ull;   /* 250 мс, макс 4 FPS   */
    uint32_t idx = 0;
    uint64_t next = now_ns();

    while ( 1 ) {
        while ( now_ns() < next ) {
            struct timespec ts = {0,1000000};
            nanosleep(&ts,NULL);
        }
        next+=period;

        FrameSlot *s = &g.slot[idx];

        if ( atomic_load_explicit( &s->st, memory_order_acquire) != FREE ) {
            fprintf( stdout, "[drop] slot %u busy\n", idx);
            idx = (idx + 1) % g.slots;
            continue;
        }

        if (!XShmGetImage(g.dpy, g.root, g.ximg[idx], 0, 0, AllPlanes)) {
            fprintf(stderr, "XShmGetImage failed\n");
            continue;
        }

        s->t_start = now_ns();
        atomic_store_explicit(&s->st, RAW_READY, memory_order_release);
        idx = (idx + 1) % g.slots;
    }
    return NULL;
}

/* ================= worker_thread ======================================= */
static void *worker_thread(void *arg)
{
    uintptr_t wid = (uintptr_t)arg;
    size_t px_cnt = (size_t)g.padded_w * g.padded_h;
    char dbg_fname[128];

    for (;;) {
        for (uint32_t i = 0; i < g.slots; ++i) {
            FrameSlot *s = &g.slot[i];
            enum slot_state exp = RAW_READY;
            if (!atomic_compare_exchange_strong(&s->st, &exp, IN_PROGRESS))
                continue;

            // 1) Квантование RGB222 + анализ uniform-блоков 8×8
            quantize_and_analyze(
                s->raw,      // ARGB8888
                s->quant,    // RGB222-byte per px
                s->color     // блоки 8×8: uniform-цвет или MIXED (0xFF)
                );

            // 2) Отладочный вывод: записать информацию по каждому блоку в файл
            // Имя файла: dbg_blocks_<slot>_<timestamp>.txt
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            snprintf(dbg_fname, sizeof(dbg_fname),
                     "dbg_blocks_%u_%ld_%09ld.txt",
                     i, ts.tv_sec, ts.tv_nsec);

            FILE *df = fopen(dbg_fname, "w");
            if (df) {
                fprintf(df, "DEBUG BLOCKS SLOT %u (worker %lu)\n", i, (unsigned long)wid);
                fprintf(df, "Screen: %dx%d, padded: %dx%d\n",
                        g.w, g.h, g.padded_w, g.padded_h);
                fprintf(df, "Blocks: %d cols × %d rows = %d total\n\n",
                        g.block_cols, g.block_rows, g.block_count);

                for (int b = 0; b < g.block_count; ++b) {
                    uint8_t c = s->color[b];
                    int col = b % g.block_cols;
                    int row = b / g.block_cols;
                    fprintf(df, "Block %3d at [%2d,%2d]: color=0x%02X %s\n",
                            b, row, col, c,
                            (c == 0xFF ? "(MIXED)" : "(UNIFORM)"));

                    if (c == 0xFF) {
                        // Если блок смешанный, вывести все 8×8 квантованных пикселей
                        fprintf(df, "  Pixels:\n    ");
                        int base_y = row * 8;
                        int base_x = col * 8;
                        for (int y = 0; y < 8; ++y) {
                            for (int x = 0; x < 8; ++x) {
                                int idx = (base_y + y) * g.padded_w + (base_x + x);
                                uint8_t q = s->quant[idx];
                                fprintf(df, "%02X ", q);
                            }
                            if (y < 7) fprintf(df, "\n    ");
                        }
                        fprintf(df, "\n");
                    }
                }
                fclose(df);
            } else {
                fprintf(stderr, "[worker %lu] failed to open debug file %s: %s\n",
                        (unsigned long)wid, dbg_fname, strerror(errno));
            }

            // 3) Ставим слот в готовое состояние
            atomic_store_explicit(&s->st, QUANT_DONE, memory_order_release);
        }
        sched_yield();
    }
    return NULL;
}

/* ================= serializer_thread =================================== */

static void *serializer_thread(void *arg)
{
    (void)arg;
    mkfifo(PIPE_PATH,0666);
    int pipe_fd=open(PIPE_PATH,O_WRONLY|O_NONBLOCK);

    char fn[64];
    while(1){
        uint64_t now=now_ns();
        for(uint32_t i=0;i<g.slots;++i){
            FrameSlot *s=&g.slot[i];

            enum slot_state exp=QUANT_DONE;
            if(atomic_compare_exchange_strong(&s->st,&exp,SERIALIZING)){
                struct timespec ts; clock_gettime(CLOCK_REALTIME,&ts);
                snprintf(fn,sizeof fn,"qimg_%ld_%09ld.qimg",
                         ts.tv_sec, ts.tv_nsec);
                write_qimg(s,fn);
                if(pipe_fd>=0) dprintf(pipe_fd,"%s\n",fn);
                atomic_store_explicit(&s->st,FREE,memory_order_release);
                continue;
            }
            enum slot_state cur=atomic_load_explicit(&s->st,memory_order_acquire);
            if((cur==IN_PROGRESS || cur==SERIALIZING)
               && now - s->t_start > STALL_NS){
                enum slot_state tmp=cur;
                if(atomic_compare_exchange_strong(&s->st,&tmp,FREE))
                    fprintf(stdout,"[stall] slot %u freed (state=%s)\n",
                            i, cur==IN_PROGRESS? "IN_PROGRESS":"SERIALIZING");
            }
        }
        sched_yield();
    }
    return NULL;
}


/* ═════════════════════════ main ══════════════════════════════════════════ */
int main(int argc,char **argv)
{
    /* Разбор аргументов: допускается только --slots=N */
    g.slots=DEFAULT_SLOTS;
    for(int i=1;i<argc;++i){
        if(strncmp(argv[i],"--slots=",8)==0){
            int v=atoi(argv[i]+8);
            if(v<2||v>MAX_SLOTS){ fprintf(stderr,"slots 2..%d\n",MAX_SLOTS); return 1; }
            g.slots=v;
        } else { fprintf(stderr,"unknown opt %s\n",argv[i]); return 1; }
    }

    /* Определяем число рабочих потоков */
    int cores=sysconf(_SC_NPROCESSORS_ONLN);
    g.workers=(cores>4)? cores-3 : 1;

    /* Инициализация X11 и общей топологии */
    init_x11();
    /* build_topology(g.w,g.h); */
    allocate_bigmem();

    /* Создаём потоки: захвата, сериализатора и воркеры */
    pthread_t cap_tid, ser_tid;
    pthread_t *w_tid=calloc(g.workers,sizeof(pthread_t));
    pthread_create(&cap_tid,NULL,capture_thread,NULL);
    pthread_create(&ser_tid,NULL,serializer_thread,NULL);
    for(uint32_t i=0;i<g.workers;++i)
        pthread_create(&w_tid[i],NULL,worker_thread,(void*)(uintptr_t)i);

    /* Ждём (но на самом деле никогда не вернётся) */
    pthread_join(cap_tid,NULL);        /* никогда не возвращается          */
    pthread_join(ser_tid,NULL);
    for(uint32_t i=0;i<g.workers;++i) pthread_join(w_tid[i],NULL);
    return 0;
}
