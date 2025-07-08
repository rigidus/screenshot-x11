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
#include <immintrin.h>    // AVX2
#include <xmmintrin.h>    // _mm_sfence()
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
#include <stdbool.h>
#include <math.h>


/* ---------- константы и вспомогалки ------------------------------------ */
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define BS            32
#define DEFAULT_SLOTS 4
#define MAX_SLOTS     8
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

enum slot_state { FREE, RAW_READY, IN_PROGRESS, QUANT_DONE, SERIALIZING };

typedef struct {
    _Atomic enum slot_state st;
    uint64_t  t_start;
    uint8_t  *raw;     // указатель на сырое изображение RGB
    uint8_t  *quant;   // RGB332 + старший transparency bit
    /* uint8_t  *color;   // массив цветов блоков uniform-цвет блока 8×8 или MIXED */
    uint8_t  *bg;      // фон каждого блока
    uint8_t  *fg;      // текст - второй цвет каждого блока
    uint8_t  *mask;    // битовая маска: 1 бит на пиксель блока – (BS*BS+7)/8 байт на блок
    int       block_count;
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


// Описывает один «остров» смешанных блоков
// Island structure to hold block indices and metadata
typedef struct Island {
    int *blocks;        // dynamic array of block indices (32×32 blocks)
    int count;          // number of blocks
    int cap;            // capacity of blocks array

    // bounding box in pixel coordinates
    int minx, miny, maxx, maxy;

    // classification results
    uint8_t bg_color;   // most frequent palette color (background)
    uint8_t fg_color;   // second most frequent (foreground/text)
    int total_pixels;   // total pixels in the island (count × BS²)
} Island;


/* ---------- объявления -------------------------------------------------- */
static void allocate_bigmem(void);
static void init_x11(void);
static void *capture_thread(void*);
static void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb);
void decompress_island_to_png(const char *fname,
                              const Island *isl,
                              int padded_w,
                              uint8_t palette[2],
                              const uint8_t *mask);
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
    size_t q_sz     = (size_t)g.padded_w * g.padded_h;          // RGB332
    size_t col_sz   = (size_t)g.block_count * sizeof(uint8_t);  // цвет каждого 8×8-блока
    size_t per_slot = q_sz + col_sz;                            // один slot
    size_t mask_sz  = ((BS*BS) + 7) / 8;
    size_t bf_sz    = (size_t)g.block_count * (1 + 1 + mask_sz);
    per_slot += bf_sz;
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
            .st = FREE,
            .raw   = (uint8_t*)g.ximg[i]->data,  // raw данные лежат в shm-образе
            .quant = base,            // следом — квант-буфер
            .bg    = base + q_sz,
            .fg    = base + q_sz + g.block_count,
            .mask  = base + q_sz + g.block_count*2,
            .block_count = g.block_count
        };
        fprintf(stderr,
                "[init slot %u] bigmem=%p base=%p quant=%p bg=%p fg=%p mask=%p\n",
                i,
                g.bigmem,
                base,
                base,
                base + q_sz,
                base + q_sz + g.block_count,
                base + q_sz + g.block_count*2
            );
        fprintf(stderr,
                "[init slot %u] g.bigmem + g.bigmem_sz = %p\n",
                i, (void*)g.bigmem + g.bigmem_sz  );
    }
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


/* ═════════════════════════ SIMD RGB888→RGB332 ═════════════════════════════ */

#define MIXED         0xFF


// Расширение 2-бит → 8-бит
static inline uint8_t expand2(uint8_t v) {
    return (uint8_t)((v << 6) | (v << 4) | (v << 2) | v);
}

// Расширение 3-бит → 8-бит
static inline uint8_t expand3(uint8_t v) {
    return (uint8_t)((v << 5) | (v << 2) | (v >> 1));
}


// CPUID‐проверка AVX2
static inline bool cpu_has_avx2(void) {
    uint32_t a, b, c, d;
    __asm__ volatile("cpuid"
                     : "=a"(a), "=b"(b), "=c"(c), "=d"(d)
                     : "a"(7), "c"(0)
        );
    return (b & (1<<5)) != 0;  // EBX[5] = AVX2
}


// SIMD‐проверка однородности 32-байтного региона
static inline bool block_uniform_avx2(
    const uint8_t *quant, int pw, int sy, int sx
    ) {
    uint8_t base = quant[sy*pw + sx];
    if (base == 0) return true;  // весь блок из нулей
    __m256i v_base = _mm256_set1_epi8((char)base);
    __m256i v_zero = _mm256_setzero_si256();
    for (int dy = 0; dy < BS; ++dy) {
        const uint8_t *row = quant + (sy + dy)*pw + sx;
        __m256i v   = _mm256_loadu_si256((const __m256i*)row);
        __m256i cmpb = _mm256_cmpeq_epi8(v, v_base);
        __m256i cmp0 = _mm256_cmpeq_epi8(v, v_zero);
        __m256i ok   = _mm256_or_si256(cmpb, cmp0);
        uint32_t mask = _mm256_movemask_epi8(ok);
        if (mask != 0xFFFFFFFFu) return false;
    }
    return true;
}

static void quantize_and_analyze(FrameSlot *slot) {
    const int W  = g.w;
    const int H  = g.h;
    const int pw = g.padded_w;
    const int bc = g.block_cols;
    const int br = g.block_rows;
    const size_t mask_sz = ((size_t)BS * BS + 7) / 8;

    // Однократный детект AVX2
    static bool inited = false;
    static bool use256;
    if (!inited) {
        use256 = cpu_has_avx2();
        inited  = true;
    }

    // 1) Векторное квантование RGB888 → RGB332
    for (int y = 0; y < H; ++y) {
        const uint8_t *row_src = slot->raw   + (size_t)y * W * 4;
        uint8_t       *row_q   = slot->quant + (size_t)y * pw;
        int x = 0;

        if (use256) {
            __m256i mR = _mm256_set1_epi32(0xE0);
            __m256i mB = _mm256_set1_epi32(0xC0);
            int lim8 = (W/8)*8;
            for (; x < lim8; x += 8) {
                __m256i pix = _mm256_loadu_si256((__m256i*)(row_src + x*4));
                __m256i r   = _mm256_and_si256(_mm256_srli_epi32(pix,16), mR);
                __m256i g2  = _mm256_and_si256(_mm256_srli_epi32(pix, 8), mR);
                __m256i b   = _mm256_and_si256(pix,               mB);
                __m256i rg  = _mm256_or_si256(r,  _mm256_srli_epi32(g2,3));
                __m256i rgb = _mm256_or_si256(rg, _mm256_srli_epi32(b,6));
                __m128i lo  = _mm256_castsi256_si128(rgb);
                __m128i hi  = _mm256_extracti128_si256(rgb,1);
                __m128i p16 = _mm_packus_epi32(lo, hi);
                __m128i p8  = _mm_packus_epi16(p16, _mm_setzero_si128());
                _mm_storel_epi64((__m128i*)(row_q + x), p8);
            }
        }

        // SSE2 путь (обрабатывает хвосты и если AVX2 нет)
        {
            __m128i mR4 = _mm_set1_epi32(0xE0);
            __m128i mB4 = _mm_set1_epi32(0xC0);
            int lim4 = (W/4)*4;
            for (; x < lim4; x += 4) {
                __m128i pix = _mm_loadu_si128((__m128i*)(row_src + x*4));
                __m128i r   = _mm_and_si128(_mm_srli_epi32(pix,16), mR4);
                __m128i g2  = _mm_and_si128(_mm_srli_epi32(pix, 8), mR4);
                __m128i b   = _mm_and_si128(pix,               mB4);
                __m128i rg  = _mm_or_si128(r, _mm_srli_epi32(g2,3));
                __m128i rgb = _mm_or_si128(rg,_mm_srli_epi32(b,6));
                __m128i p16 = _mm_packus_epi32(rgb, _mm_setzero_si128());
                __m128i p8  = _mm_packus_epi16(p16, _mm_setzero_si128());
                *(uint32_t*)(row_q + x) = _mm_cvtsi128_si32(p8);
            }
        }

        // Скаляный хвост + паддинг
        for (; x < W; ++x) {
            uint8_t R = row_src[x*4+2];
            uint8_t G = row_src[x*4+1];
            uint8_t B = row_src[x*4+0];
            uint8_t R3 = R >> 5, G3 = G >> 3, B2 = B >> 6;
            row_q[x] = (uint8_t)((R3<<5)|(G3<<2)|B2);
        }
        for (; x < pw; ++x) {
            row_q[x] = 0;
        }
    }

    _mm_sfence();  // дождаться store

    /* dump_png_rgb("dbg_quant_full.png", W, H, slot->quant_expanded); */



    // 2) Анализ блоков 32×32 → bg/fg/mask
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {
            int idx = by*bc + bx;
            uint8_t *bg = &slot->bg[idx];
            uint8_t *fg = &slot->fg[idx];
            uint8_t *m  = slot->mask + idx*mask_sz;

            // собираем гистограмму
            int hist[256] = {0};
            int x0   = bx * BS;
            int y0   = by * BS;
            int w_blk = MIN(BS, W - x0);
            int h_blk = MIN(BS, H - y0);
            int total = w_blk * h_blk;
            for (int dy = 0; dy < BS; ++dy) {
                for (int dx = 0; dx < BS; ++dx) {
                    uint8_t q = slot->quant[(by*BS+dy)*pw + (bx*BS+dx)];
                    hist[q]++;
                    /* total++; */
                }
            }

            // находим самый частый цвет
            int best = 0;
            for (int c = 1; c < 256; ++c)
                if (hist[c] > hist[best]) best = c;
            *bg = (uint8_t)best;
            *fg = (uint8_t)best;

            int second = -1, sc = -1;
            if (hist[best] == total) {
                // uniform — очиcтим маску
                memset(m, 0, mask_sz);
            } else {
                // mixed — ищем второй по частоте
                for (int c = 0; c < 256; ++c) {
                    if (c == best) continue;
                    if (hist[c] > sc) { sc = hist[c]; second = c; }
                }
                if (second < 0) {
                    second = best;
                    sc = -1;
                }
                *fg = (uint8_t)second;

                // строим маску
                memset(m, 0, mask_sz);
                int bit = 0;
                for (int dy = 0; dy < BS; ++dy) {
                    for (int dx = 0; dx < BS; ++dx, ++bit) {
                        uint8_t q = slot->quant[(by*BS+dy)*pw + (bx*BS+dx)];
                        if (q != best)
                            m[bit>>3] |= (uint8_t)(1u << (bit&7));
                    }
                }
            }

            /* DBG */
            /* fprintf(stderr, "Hist block %d (best = 0x%02X: %d) (second = 0x%02X: %d)\n", */
            /*         idx, best, hist[best], second, second==-1 ? -1 : hist[second]); */
            /* for (int c = 0; c < 256; ++c) { */
            /*     if (hist[c] > 0) { // только те цвета что встречаются хоть раз */
            /*         fprintf(stderr, "  color 0x%02X: %d\n", c, hist[c]); */
            /*     } */
            /* } */

            /* // DBG */
            /* // Выводим бит за битом 32×32 */
            /* fprintf(stderr, "\nMask for block %d:\n", idx); */
            /* for (int bit = 0; bit < BS*BS; ++bit) { */
            /*     int byte = bit >> 3; */
            /*     int bpos = bit & 7; */
            /*     int v = (m[byte] >> bpos) & 1; */
            /*     fputc(v ? '1' : '0', stderr); */
            /*     // по окончании каждой строки 32 символа — перевод строки */
            /*     if ((bit+1) % BS == 0) */
            /*         fputc('\n', stderr); */
            /* } */
            /* fputc('\n', stderr); */

        }
    }
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
        /* uint8_t c = s->color[b]; */
        /* if (fwrite(&c, 1, 1, fp) != 1) { */
        /*     perror("fwrite block_color"); */
        /*     fclose(fp); */
        /*     return; */
        /* } */

        /* // Если блок смешанный (0xFF), записываем 8×8 квантованных пикселей */
        /* if (c == 0xFF) { */
        /*     int row = b / g.block_cols; */
        /*     int col = b % g.block_cols; */
        /*     for (int y = 0; y < 8; ++y) { */
        /*         // смещение в quant-буфере по строкам с учётом паддинга */
        /*         size_t offset = (size_t)(row * 8 + y) * g.padded_w + (col * 8); */
        /*         if (fwrite(s->quant + offset, 1, 8, fp) != 8) { */
        /*             perror("fwrite block_pixels"); */
        /*             fclose(fp); */
        /*             return; */
        /*         } */
        /*     } */
        /* } */
    }

    fclose(fp);
}

/* ═════════════════════════ потоки ════════════════════════════════════════ */

static void *capture_thread(void *arg)
{
    (void)arg;
    const uint64_t period = 250000000ull;  // 250 ms → max 4 FPS
    uint32_t idx = 0;
    uint64_t next = now_ns();

    while (1) {
        // Ждём до следующего кадра
        while (now_ns() < next) {
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000000 };
            nanosleep(&ts, NULL);
        }
        next += period;

        FrameSlot *s = &g.slot[idx];
        // Если слот занят — дропаем и идём дальше
        if (atomic_load_explicit(&s->st, memory_order_acquire) != FREE) {
            fprintf(stdout, "[drop] slot %u busy\n", idx);
            idx = (idx + 1) % g.slots;
            continue;
        }

        // Захватываем изображение прямо в XShm-образ слота
        if (!XShmGetImage(g.dpy, g.root, g.ximg[idx], 0, 0, AllPlanes)) {
            fprintf(stderr, "XShmGetImage failed\n");
            idx = (idx + 1) % g.slots;
            continue;
        }

        // Помечаем время начала обработки и переводим слот в RAW_READY
        s->t_start = now_ns();
        atomic_store_explicit(&s->st, RAW_READY, memory_order_release);

        // Переходим к следующему слоту
        idx = (idx + 1) % g.slots;
    }

    // никогда не возвращаемся
    return NULL;
}


/* ================= worker_thread ======================================= */

#define EXPAND2(v) ((uint8_t)((v<<6)|(v<<4)|(v<<2)|(v)))


/**
 * dump_png_rgb
 *
 * Сохраняет буфер RGB888 (3 байта на пиксель, упаковано по строкам подряд)
 * в PNG-файл с именем fname.
 *
 * @param fname  имя выходного файла
 * @param W      ширина изображения в пикселях
 * @param H      высота изображения в пикселях
 * @param rgb    указатель на буфер размера W*H*3 байт
 */
static void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb)
{
    FILE *fp = fopen(fname, "wb");
    if (!fp) {
        perror("dump_png_rgb: fopen");
        return;
    }

    png_structp png = png_create_write_struct(
        PNG_LIBPNG_VER_STRING, NULL, NULL, NULL
        );
    if (!png) {
        fprintf(stderr, "dump_png_rgb: png_create_write_struct failed\n");
        fclose(fp);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        fprintf(stderr, "dump_png_rgb: png_create_info_struct failed\n");
        png_destroy_write_struct(&png, (png_infopp)NULL);
        fclose(fp);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        fprintf(stderr, "dump_png_rgb: libpng longjmp error\n");
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return;
    }

    png_init_io(png, fp);

    // Настройки IHDR
    png_set_IHDR(
        png, info,
        W, H,
        8,                         // глубина бит на канал
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT
        );
    png_write_info(png, info);

    // Запись строк
    for (int y = 0; y < H; ++y) {
        png_bytep row_ptr = (png_bytep)(rgb + (size_t)y * W * 3);
        png_write_row(png, row_ptr);
    }

    png_write_end(png, info);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
}



/**
 * Находит все связанные «острова» блоков, у которых block_color==MIXED.
 * Возвращает массив Island[ *out_n ], сам массив и поля need to be freed by caller.
 */
static Island* detect_mixed_islands(const uint8_t *block_color,
                                    int rows, int cols,
                                    int *out_n)
{
    // Вычисляется общее число блоков N = rows * cols.
    int N = rows * cols;
    // Создаётся массив vis[N] (булев), чтобы помечать, какие блоки уже посещены.
    bool *vis = calloc(N, sizeof(bool));
    // Заводится динамический массив list для хранения найденных островов (Island),
    Island *list = NULL;
    // а также счётчики list_n (сколько уже добавлено) и list_cap (ёмкость).
    int  list_n = 0, list_cap = 0;

    // Вспомогательная очередь для BFS
    int *queue = malloc(N * sizeof(int));

    // Основной цикл по всем блокам
    for (int idx = 0; idx < N; ++idx) {
        // Мы ищем каждый блок, у которого цвет MIXED и который ещё не был посещён.
        if (vis[idx] || block_color[idx] != MIXED) continue;

        // начинаем новый остров
        if (list_n == list_cap) {
            list_cap = list_cap ? list_cap*2 : 8;
            list = realloc(list, list_cap * sizeof(Island));
        }
        Island *isl = &list[list_n++];
        isl->count = 0;
        isl->cap   = 0;
        isl->blocks = NULL;
        // Это будет наш очередной остров — набор смежных «смешанных» блоков.

        // BFS из idx (BFS (поиск в ширину) из стартового блока)
        int qh=0, qt=0;
        queue[qt++] = idx;
        vis[idx] = true;

        while (qh < qt) {
            int cur = queue[qh++];

            // добавляем в остров
            if (isl->count == isl->cap) {
                isl->cap = isl->cap ? isl->cap*2 : 16;
                isl->blocks = realloc(isl->blocks, isl->cap * sizeof(int));
            }
            isl->blocks[isl->count++] = cur;

            int r = cur / cols, c = cur % cols;
            // 4-соседи
            const int dr[4] = {-1,1,0,0}, dc[4] = {0,0,-1,1};
            for (int k=0; k<4; ++k) {
                int nr = r + dr[k], nc = c + dc[k];
                if (nr<0||nr>=rows||nc<0||nc>=cols) continue;
                int ni = nr*cols + nc;
                if (!vis[ni] && block_color[ni]==MIXED) {
                    vis[ni]=true;
                    queue[qt++]=ni;
                }
            }
        }
        // После того, как BFS закончился, мы полностью собрали один
        // остров: все «смешанные» блоки, до которых можно добраться
        // из стартового, помечены и добавлены. Цикл for(idx…)
        // продолжит искать следующие непосещённые «смешанные» блоки
        // и так до конца.
    }

    free(vis);
    free(queue);
    *out_n = list_n;
    return list;
}

/**
 * signature: palette и counts выделяются внутри и возвращаются через указатели
 * асимптотика O(BS² × number_of_blocks_in_island)
 */
static void collect_palette_hist(
    const uint8_t *quant,
    int padded_w,
    const Island *isl,
    uint8_t **out_palette,
    int    *out_palette_n,
    int   **out_counts
    ) {

    // отслеживаем, какие цвета уже добавлены в палитру
    // seen[q] = true означает, что цвет q уже записан в palette.
    bool seen[256]={0};
    // counts[q] будет считать количество пикселей цвета q.
    int *counts = calloc(256, sizeof(int));
    uint8_t *palette = malloc(256);         // максимум 256 возможных значений
    int pcount = 0;                         // сколько цветов в палитре

    // Перебор блоков внутри острова
    for (int bi = 0; bi < isl->count; ++bi) {
        // Каждый элемент isl->blocks[bi] — это номер 8×8-блока внутри
        // исходного изображения.  Из него вычисляют координаты
        // верхнего-левого угла блока (y0, x0).
        int b = isl->blocks[bi];
        int br = b / g.block_cols, bc = b % g.block_cols;
        int y0 = br*BS, x0 = bc*BS;
        // Перебор всех пикселей в блоке
        for (int dy=0; dy<BS; ++dy) {
            for (int dx=0; dx<BS; ++dx) {
                // Берём каждый квантованный байт q из буфера quant
                // (ширина с учётом выравнивания — padded_w).
                uint8_t q = quant[(y0+dy)*padded_w + (x0+dx)];
                /* if (q & TRANSP_BIT) continue; // пропускаем «прозрачные» пиксели */
                if (!seen[q]) {
                    // первый раз, когда цвет q встречается,
                    // добавляем его в массив palette и отмечаем в seen.
                    seen[q] = true;
                    palette[pcount++] = q;
                }
                // каждый раз, когда попадается пиксель цвета q,
                // инкрементируем counts[q]
                counts[q] += 1;
            }
        }
    }
    // указатель на динамически выделенный массив из pcount байт,
    // каждый — это уникальный цвет.
    *out_palette   = palette;
    *out_palette_n = pcount; // число этих уникальных цветов.
    // указатель на массив из 256 int, где counts[q] - сколько всего
    // пикселей цвета q в острове
    *out_counts    = counts;
}





/**
 * Генерация «полупрозрачного» цвета из индекса
 */
static void pick_island_color(int idx, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Простой хеш: меняем компоненты по-разному
    *r = (uint8_t)( 50 + (idx * 37) % 156 );
    *g = (uint8_t)(100 + (idx * 59) % 156 );
    *b = (uint8_t)(150 + (idx * 83) %  76 );
}

/**
 * Рисует огибающую рамку заданного острова (сплошной прямоугольник)
 */
static void draw_island_bbox(uint8_t *rgb, int W, int H,
                             const Island *isl, int padded_w,
                             uint8_t rr, uint8_t gg, uint8_t bb)
{
    // найдем min/max по x и y среди блоков
    int minx = W, miny = H, maxx = 0, maxy = 0;
    for (int i = 0; i < isl->count; ++i) {
        int b = isl->blocks[i];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int x0 = bx * BS, y0 = by * BS;
        minx = x0 < minx ? x0 : minx;
        miny = y0 < miny ? y0 : miny;
        maxx = (x0 + BS - 1) > maxx ? x0 + BS - 1 : maxx;
        maxy = (y0 + BS - 1) > maxy ? y0 + BS - 1 : maxy;
    }
    // ——— Обрезаем рамку по границам изображения ———
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx >= W) maxx = W - 1;
    if (maxy >= H) maxy = H - 1;

    // топ и низ
    for (int x = minx; x <= maxx; ++x) {
        size_t p1 = miny * W + x;
        size_t p2 = maxy * W + x;
        rgb[p1*3+0]=rgb[p1*3+1]=rgb[p1*3+2]=0;
        rgb[p2*3+0]=rgb[p2*3+1]=rgb[p2*3+2]=0;
    }
    // лево и право
    for (int y = miny; y <= maxy; ++y) {
        size_t p1 = (size_t)y * W + minx;
        size_t p2 = (size_t)y * W + maxx;
        rgb[p1*3+0]=rgb[p1*3+1]=rgb[p1*3+2]=0;
        rgb[p2*3+0]=rgb[p2*3+1]=rgb[p2*3+2]=0;
    }
    // заливка полупрозрачным цветом (50% смешение с фоном)
    for (int i = 0; i < isl->count; ++i) {
        int b = isl->blocks[i];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int x0 = bx * BS, y0 = by * BS;
        for (int dy = 0; dy < BS; ++dy) {
            int y = y0 + dy; if (y >= H) break;
            for (int dx = 0; dx < BS; ++dx) {
                int x = x0 + dx; if (x >= W) break;
                size_t p = (size_t)y * W + x;
                // оригинальный фон
                uint8_t fr = rgb[p*3+0], fg = rgb[p*3+1], fb = rgb[p*3+2];
                // полупрозрачный слой
                rgb[p*3+0] = (fr + rr) / 2;
                rgb[p*3+1] = (fg + gg) / 2;
                rgb[p*3+2] = (fb + bb) / 2;
            }
        }
    }
    // 6) дополнительная рамка цветом #7F7F7F
    uint8_t gr = 0x7F, gg2 = 0x7F, gb = 0x7F;
    // верх/низ
    for (int x = minx; x <= maxx; ++x) {
        size_t p1 = miny * W + x;
        size_t p2 = maxy * W + x;
        rgb[p1*3+0] = gr; rgb[p1*3+1] = gg2; rgb[p1*3+2] = gb;
        rgb[p2*3+0] = gr; rgb[p2*3+1] = gg2; rgb[p2*3+2] = gb;
    }
    // лево/право
    for (int y = miny; y <= maxy; ++y) {
        size_t p1 = (size_t)y * W + minx;
        size_t p2 = (size_t)y * W + maxx;
        rgb[p1*3+0] = gr; rgb[p1*3+1] = gg2; rgb[p1*3+2] = gb;
        rgb[p2*3+0] = gr; rgb[p2*3+1] = gg2; rgb[p2*3+2] = gb;
    }
}




// расширение RGB332 → [0..255]
static inline void expand_rgb332(uint8_t q, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = (q >> 4) & 0x3; *r = (*r<<6)|(*r<<4)|(*r<<2)|*r;
    *g = (q >> 2) & 0x3; *g = (*g<<6)|(*g<<4)|(*g<<2)|*g;
    *b = (q     ) & 0x3; *b = (*b<<6)|(*b<<4)|(*b<<2)|*b;
}

/// Сжимает один остров в 2-цветную палитру + битовую маску.
/// quant — полный буфер [padded_h][padded_w]; isl — остров;
/// palette[2] — выходная палитра; mask — битовая маска (выделяется внутри);
/// out_mask_bytes — размер в байтах mask.
void compress_island_2col(const uint8_t *quant, int padded_w,
                          const Island *isl,
                          uint8_t palette[2],
                          uint8_t **mask, int *out_mask_bytes)
{
    // 1) Собрать гистограмму (уже есть counts[]) и palette[]
    uint8_t *pl; int pn; int *counts;
    collect_palette_hist(quant, padded_w, isl, &pl, &pn, &counts);

    // 2) Найти c0 = наиболее частый, c1 = самый удалённый
    int best0 = 0;
    for (int i = 1; i < pn; ++i)
        if (counts[pl[i]] > counts[pl[best0]])
            best0 = i;
    uint8_t c0 = pl[best0];

    double maxd = -1;
    int best1 = -1;
    uint8_t r0,g0,b0;
    expand_rgb332(c0,&r0,&g0,&b0);
    for (int i = 0; i < pn; ++i) if (i != best0) {
            uint8_t ri,gi,bi;
            expand_rgb332(pl[i], &ri, &gi, &bi);
            double d = (r0-ri)*(r0-ri) + (g0-gi)*(g0-gi) + (b0-bi)*(b0-bi);
            if (d > maxd) { maxd = d; best1 = i; }
        }
    uint8_t c1 = (best1>=0 ? pl[best1] : c0^0x3F); // если вдруг палитра из 1 цвета

    // 3) Кластерная принадлежность для всех возможных q
    uint8_t cluster[256];
    for (int i = 0; i < 256; ++i) cluster[i] = 0;
    // центр 0
    cluster[c0] = 0;
    // центр 1
    cluster[c1] = 1;
    // остальные
    for (int i = 0; i < pn; ++i) {
        int q = pl[i];
        if (q == c0 || q == c1) continue;
        uint8_t ri,gi,bi;
        expand_rgb332(q,&ri,&gi,&bi);
        double d0 = (r0 - ri)*(r0 - ri) + (g0 - gi)*(g0 - gi) + (b0 - bi)*(b0 - bi);
        // раскладка для c1
        uint8_t r1,g1,b1; expand_rgb332(c1,&r1,&g1,&b1);
        double d1 = (r1 - ri)*(r1 - ri) + (g1 - gi)*(g1 - gi) + (b1 - bi)*(b1 - bi);
        cluster[q] = (d1 < d0) ? 1 : 0;
    }

    // 4) Формируем битовую маску
    int pixels = isl->count * BS * BS;
    int bytes = (pixels + 7) / 8;
    uint8_t *m = calloc(bytes, 1);

    int bitpos = 0;
    for (int bi = 0; bi < isl->count; ++bi) {
        int b = isl->blocks[bi];
        int by = b / g.block_cols;
        int bx = b % g.block_cols;
        int x0 = bx*BS, y0 = by*BS;
        for (int dy = 0; dy < BS; ++dy) {
            for (int dx = 0; dx < BS; ++dx) {
                int q_idx = (y0+dy)*padded_w + (x0+dx);
                int cl = cluster[ quant[q_idx] ];
                if (cl) m[bitpos>>3] |= (1 << (bitpos&7));
                ++bitpos;
            }
        }
    }

    // Выдаём
    palette[0]      = c0;
    palette[1]      = c1;
    *mask           = m;
    *out_mask_bytes = bytes;

    free(pl);
    free(counts);
}

void compress_island_2col_fast(
    const uint8_t   *quant,
    int              padded_w,
    const Island    *isl,
    const uint8_t   *palette,
    int              palette_n,
    const int       *counts,
    uint8_t          out_pal2[2],
    uint8_t        **out_mask,
    int             *out_mask_bytes
    ) {
    // 1) выбрать два опорных цвета: наиболее частый (c0) и максимально удалённый от него (c1)
    // -------------------------------------------------------
    // находим индекс самого частого
    int best0 = 0;
    for (int i = 1; i < palette_n; ++i) {
        if (counts[ palette[i] ] > counts[ palette[best0] ])
            best0 = i;
    }
    uint8_t c0 = palette[best0];

    // расширим c0 в RGB888 для вычисления дистанций
    uint8_t r0,g0,b0;
    expand_rgb332(c0, &r0, &g0, &b0);

    // ищем в палитре цвет, максимально удалённый от c0
    double maxd = -1.0;
    int best1 = -1;
    for (int i = 0; i < palette_n; ++i) {
        if (i == best0) continue;
        uint8_t ri,gi,bi;
        expand_rgb332(palette[i], &ri, &gi, &bi);
        double d = (r0 - ri)*(r0 - ri)
            + (g0 - gi)*(g0 - gi)
            + (b0 - bi)*(b0 - bi);
        if (d > maxd) {
            maxd = d;
            best1 = i;
        }
    }
    uint8_t c1 = (best1 >= 0 ? palette[best1] : (c0 ^ 0x3F));

    out_pal2[0] = c0;
    out_pal2[1] = c1;

    // 2) построить кластеризацию: для каждого q из палитры определить, к какому кластеру (0 или 1) он ближе
    // -------------------------------------------------------------------------------------------------
    uint8_t cluster[256] = {0};
    cluster[c0] = 0;
    cluster[c1] = 1;

    for (int i = 0; i < palette_n; ++i) {
        uint8_t q = palette[i];
        if (q == c0 || q == c1) continue;
        // расстояния до c0 и c1
        uint8_t ri,gi,bi, r1,g1,b1;
        expand_rgb332(q,   &ri,&gi,&bi);
        expand_rgb332(c1, &r1,&g1,&b1);
        double d0 = (r0 - ri)*(r0 - ri) + (g0 - gi)*(g0 - gi) + (b0 - bi)*(b0 - bi);
        double d1 = (r1 - ri)*(r1 - ri) + (g1 - gi)*(g1 - gi) + (b1 - bi)*(b1 - bi);
        cluster[q] = (d1 < d0) ? 1 : 0;
    }

    // 3) сформировать битовую маску: для каждого пикселя острова пишем 0/1
    // -----------------------------------------------------------------
    int pixels = isl->count * BS * BS;
    int bytes  = (pixels + 7) / 8;
    uint8_t *m = calloc(bytes, 1);
    int bitpos = 0;

    for (int bi = 0; bi < isl->count; ++bi) {
        int b = isl->blocks[bi];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int y0 = by * BS, x0 = bx * BS;

        for (int dy = 0; dy < BS; ++dy) {
            for (int dx = 0; dx < BS; ++dx, ++bitpos) {
                int idx = (y0 + dy) * padded_w + (x0 + dx);
                uint8_t q = quant[idx];
                if (cluster[q])
                    m[bitpos >> 3] |= (1 << (bitpos & 7));
            }
        }
    }

    *out_mask        = m;
    *out_mask_bytes  = bytes;
}


/**
 * Упрощённая сжатая версия «острова»:
 *   - на выходе два цвета: фон и «всё остальное»,
 *   - битовая маска: 0=фон, 1=любой другой цвет.
 *
 * @param quant           входной буфер RGB332 [padded_h][padded_w]
 * @param padded_w        ширина quant с паддингом
 * @param isl             остров (список блоков)
 * @param palette         уже собранная palette[palette_n]
 * @param palette_n       длина palette
 * @param counts          гистограмма counts[0..255]
 *
 * @param out_pal2        uint8_t[2]: [0]=фон, [1]=второй по частоте или fallback
 * @param out_mask        указатель на битовую маску (malloc внутри)
 * @param out_mask_bytes  размер маски в байтах
 */
void compress_island_2col_simple(
    const uint8_t   *quant,
    int              padded_w,
    const Island    *isl,
    const uint8_t   *palette,
    int              palette_n,
    const int       *counts,
    uint8_t          out_pal2[2],
    uint8_t        **out_mask,
    int             *out_mask_bytes
    ) {
    // 1) Находим фон: наиболее частый цвет
    int best0 = 0;
    for (int i = 1; i < palette_n; ++i) {
        if (counts[ palette[i] ] > counts[ palette[best0] ])
            best0 = i;
    }
    uint8_t c0 = palette[best0];

    // 2) Второй по частоте цвет (или fallback)
    int best1 = -1;
    for (int i = 0; i < palette_n; ++i) {
        if (i == best0) continue;
        if (best1 < 0 || counts[ palette[i] ] > counts[ palette[best1] ])
            best1 = i;
    }
    uint8_t c1 = (best1 >= 0 ? palette[best1] : (c0 ^ 0x3F));

    out_pal2[0] = c0;
    out_pal2[1] = c1;

    // 3) Строим битовую маску: 0 для фоновых пикселей, 1 для любых других
    int pixels = isl->count * BS * BS;
    int bytes  = (pixels + 7) / 8;
    uint8_t *mask = calloc(bytes, 1);
    int bitpos = 0;

    for (int bi = 0; bi < isl->count; ++bi) {
        int b  = isl->blocks[bi];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int y0 = by * BS, x0 = bx * BS;

        for (int dy = 0; dy < BS; ++dy) {
            for (int dx = 0; dx < BS; ++dx, ++bitpos) {
                uint8_t q = quant[(y0 + dy) * padded_w + (x0 + dx)];
                if (q != c0) {
                    mask[bitpos >> 3] |= (1 << (bitpos & 7));
                }
            }
        }
    }

    *out_mask        = mask;
    *out_mask_bytes  = bytes;
}


/**
 * decompress_island_to_png
 *
 * Разворачивает сжатый остров из 2-цветной палитры + битовой маски
 * и сохраняет его как PNG-файл, ограниченный минимальным прямоугольником острова.
 *
 * @param fname     имя выходного PNG
 * @param isl       указатель на Island (список блоков)
 * @param padded_w  ширина квантованного буфера (с паддингом)
 * @param palette   два байта RGB332-кода: palette[0] и palette[1]
 * @param mask      битовая маска длины = island.count*64 пикселей
 */
void decompress_island_to_png(
    const char      *fname,
    const Island    *isl,
    int              padded_w,
    uint8_t          palette[2],
    const uint8_t   *mask
    ) {
    // 1) вычисляем минимальный ограничивающий прямоугольник в пикселях
    int minx = g.w, miny = g.h, maxx = 0, maxy = 0;
    for (int i = 0; i < isl->count; ++i) {
        int b  = isl->blocks[i];
        int by = b / g.block_cols;
        int bx = b % g.block_cols;
        int x0 = bx * BS;
        int y0 = by * BS;
        minx = x0 < minx ? x0 : minx;
        miny = y0 < miny ? y0 : miny;
        maxx = (x0 + BS - 1) > maxx ? x0 + BS - 1 : maxx;
        maxy = (y0 + BS - 1) > maxy ? y0 + BS - 1 : maxy;
    }
    // Обрезаем по границам экрана
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx >= g.w)  maxx = g.w - 1;
    if (maxy >= g.h)  maxy = g.h - 1;

    int W = maxx - minx + 1;
    int H = maxy - miny + 1;

    // 2) выделяем RGB-буфер и заполняем белым фоном
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) {
        perror("decompress_island_to_png: malloc");
        return;
    }
    memset(rgb, 255, (size_t)W * H * 3);

    // 3) развернуть биты в палитру
    int bitpos = 0;
    for (int i = 0; i < isl->count; ++i) {
        int b  = isl->blocks[i];
        int by = b / g.block_cols;
        int bx = b % g.block_cols;
        int x0 = bx * BS;
        int y0 = by * BS;

        for (int dy = 0; dy < BS; ++dy) {
            int yy = y0 + dy;
            if (yy < miny || yy > maxy) {
                bitpos += BS;
                continue;
            }
            for (int dx = 0; dx < BS; ++dx, ++bitpos) {
                int xx = x0 + dx;
                if (xx < minx || xx > maxx) continue;

                // какой кластер: 0 или 1
                int cluster = (mask[bitpos >> 3] >> (bitpos & 7)) & 1;
                uint8_t q = palette[cluster];
                // расширяем RGB332 → RGB888
                uint8_t r2 = (q >> 5) & 7;
                uint8_t g2 = (q >> 2) & 7;
                uint8_t b2 =  q       & 3;
                uint8_t r8 = expand3(r2);
                uint8_t g8 = expand3(g2);
                uint8_t b8 = expand2(b2);
                int px = xx - minx;
                int py = yy - miny;
                size_t off = ((size_t)py * W + px) * 3;
                rgb[off + 0] = r8;
                rgb[off + 1] = g8;
                rgb[off + 2] = b8;
            }
        }
    }

    // 4) сохраняем PNG
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
}


// Debug: dump raw BGRA→RGB buffer as PNG
static void debug_dump_raw(int slot,
                           const uint8_t *raw_bgra,
                           int w, int h) {
    uint8_t *rgb = malloc((size_t)w * h * 3);
    if (!rgb) die("debug_dump_raw: malloc failed");
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            size_t idx = (size_t)y * w + x;
            const uint8_t *px = raw_bgra + idx * 4;
            rgb[idx*3 + 0] = px[2]; // R
            rgb[idx*3 + 1] = px[1]; // G
            rgb[idx*3 + 2] = px[0]; // B
        }
    }
    char fn[64];
    struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "dbg_raw_%d_%ld_%09ld.png",
             slot, ts.tv_sec, ts.tv_nsec);
    dump_png_rgb(fn, w, h, rgb);
    free(rgb);
}




// Debug: dump quantized RGB332 buffer as RGB888 PNG
static void debug_dump_quant(int slot, const uint8_t *quant,int padded_w) {
    uint8_t *rgb = malloc((size_t)g.w * g.h * 3);
    if (!rgb) die("debug_dump_quant: malloc failed");
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            uint8_t q  = quant[y * padded_w + x];
            uint8_t r3 = (q >> 5) & 0x7;
            uint8_t g3 = (q >> 2) & 0x7;
            uint8_t b2 =  q        & 0x3;
            size_t idx = (size_t)y * g.w + x;
            rgb[idx*3+0] = expand3(r3);
            rgb[idx*3+1] = expand3(g3);
            rgb[idx*3+2] = expand2(b2);
        }
    }
    char fn[64];
    struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "dbg_quant_%d_%ld_%09ld.png",
             slot, ts.tv_sec, ts.tv_nsec);
    dump_png_rgb(fn, g.w, g.h, rgb);
    free(rgb);
}

// Рисует отладочное изображение, где каждый 32×32-блок:
//  - uniform (mask all zero): заливают bg, рисуют белые/чёрные рамки и диагонали,
//  - mixed: внутри блока по-пиксельно 0→bg, 1→fg.
//-----------------------------------------------------------------------------
static void dump_filled_blocks_png(const char *fname, const FrameSlot *slot)
{
    const int W   = g.w;
    const int H   = g.h;
    const int bc  = g.block_cols;
    const int br  = g.block_rows;
    const int pw  = g.padded_w;
    const size_t mask_sz = ((size_t)BS*BS + 7)/8;

    // 1) Серый фон
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { perror("dump_filled_blocks_png"); return; }
    memset(rgb, 127, (size_t)W * H * 3);

    // 2) По блокам 32×32
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {

            int idx = by*bc + bx;
            uint8_t bgc = slot->bg[idx];
            uint8_t fgc = slot->fg[idx];
            uint8_t *m   = slot->mask + (size_t)idx * mask_sz;

            // распаковать 2 цвета в RGB888
            uint8_t br8 = expand3((bgc>>5)&7),
                bg8 = expand3((bgc>>2)&7),
                bb8 = expand2( bgc     &3);
            uint8_t fr8 = expand3((fgc>>5)&7),
                fg8 = expand3((fgc>>2)&7),
                fb8 = expand2( fgc     &3);

            int x0 = bx * BS, y0 = by * BS;
            int w_blk = MIN(BS, W - x0), h_blk = MIN(BS, H - y0);

            // uniform-блок, если цвета совпадают
            if (bgc == fgc) {
                // залить цветом bg
                for (int dy = 0; dy < h_blk; ++dy) {
                    for (int dx = 0; dx < w_blk; ++dx) {
                        size_t p = ((size_t)(y0+dy)*W + (x0+dx)) * 3;
                        rgb[p+0] = br8;
                        rgb[p+1] = bg8;
                        rgb[p+2] = bb8;
                    }
                }
                // рамки и диагонали (как раньше)
                // — верхняя белая
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = (size_t)y0 * W + (x0+dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                }
                // — левая белая
                for (int dy = 0; dy < h_blk; ++dy) {
                    size_t p = (size_t)(y0+dy) * W + x0;
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                }
                // — нижняя чёрная
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = (size_t)(y0 + h_blk - 1) * W + (x0+dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                }
                // — правая чёрная
                for (int dy = 0; dy < h_blk; ++dy) {
                    size_t p = (size_t)(y0+dy) * W + (x0 + w_blk - 1);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                }
                // диагональ ↘ чёрная
                {
                    int dx=0, dy=0, err=0;
                    while (dx < w_blk && dy < h_blk) {
                        size_t p = (size_t)(y0+dy)*W + (x0+dx);
                        rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                        err += h_blk;
                        if (err >= w_blk) { err -= w_blk; ++dy; }
                        ++dx;
                    }
                }
                // диагональ ↙ белая
                {
                    int dx=0, dy=0, err=0;
                    while (dx < w_blk && dy < h_blk) {
                        size_t p = (size_t)(y0+dy)*W
                            + (x0 + w_blk - 1 - dx);
                        rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                        err += h_blk;
                        if (err >= w_blk) { err -= w_blk; ++dy; }
                        ++dx;
                    }
                }
            }
            else {
                // mixed-блок: по-пиксельно 0→bg, 1→fg
                int bit = 0;
                for (int dy = 0; dy < h_blk; ++dy) {
                    for (int dx = 0; dx < w_blk; ++dx, ++bit) {
                        bool one = (m[bit>>3] >> (bit&7)) & 1;
                        size_t p = ((size_t)(y0+dy)*W + (x0+dx)) * 3;
                        rgb[p+0] = one ? fr8 : br8;
                        rgb[p+1] = one ? fg8 : bg8;
                        rgb[p+2] = one ? fb8 : bb8;
                    }
                    // для безопасности, если w_blk<BS, продвинем бит на остаток
                    bit += (BS - w_blk);
                }
            }
        }
    }

    // 3) Сохраняем
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
}



/**
 * Debug: dump filled vs unfilled 32×32 blocks visualization
 */
static void debug_dump_filled(int slot_idx, const FrameSlot *slot) {
    char fname[64];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fname, sizeof(fname), "dbg_fill_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_filled_blocks_png(fname, slot);
}



/** вычисляет пиксельный bounding box для острова,
 *  исходя из списка 32×32 блоков.
 */
static void compute_island_bbox(Island *isl) {
    int bs = BS;
    isl->minx = INT_MAX;
    isl->miny = INT_MAX;
    isl->maxx = 0;
    isl->maxy = 0;
    for (int i = 0; i < isl->count; ++i) {
        int b = isl->blocks[i];
        int by = b / g.block_cols;
        int bx = b % g.block_cols;
        int x0 = bx * bs;
        int y0 = by * bs;
        isl->minx = MIN(isl->minx, x0);
        isl->miny = MIN(isl->miny, y0);
        isl->maxx = MAX(isl->maxx, x0 + bs - 1);
        isl->maxy = MAX(isl->maxy, y0 + bs - 1);
    }
}


/**
 *  Check if two bounding boxes intersect or touch
 */
static inline bool bbox_intersect(const Island *a, const Island *b) {
    return !(a->maxx < b->minx
             || b->maxx < a->minx
             || a->maxy < b->miny
             || b->maxy < a->miny);
}

/**
 * Merges any islands whose bounding boxes intersect (or one contains the other).
 *
 * @param list        Array of Island structures (each with valid minx, miny, maxx, maxy).
 * @param n_islands   Pointer to the number of islands in list; updated on return.
 * @returns           Pointer to the (possibly reallocated) list array.
 */
static Island* merge_islands(Island *list, int *n_islands) {
    int n = *n_islands;
    bool merged_any;

    do {
        merged_any = false;

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                // If the bounding boxes overlap or touch
                if (bbox_intersect(&list[i], &list[j])) {
                    Island *a = &list[i];
                    Island *b = &list[j];

                    // Ensure capacity for merged blocks
                    int new_count = a->count + b->count;
                    if (new_count > a->cap) {
                        a->cap = new_count;
                        a->blocks = realloc(a->blocks, a->cap * sizeof(int));
                        if (!a->blocks) die("merge_islands: realloc failed");
                    }

                    // Append b's blocks into a
                    memcpy(a->blocks + a->count,
                           b->blocks,
                           b->count * sizeof(int));
                    a->count = new_count;

                    // Recompute a's bounding box
                    compute_island_bbox(a);

                    // Free b's block array
                    free(b->blocks);

                    // Remove b from the list by shifting subsequent entries left
                    memmove(&list[j],  &list[j+1],  (n - j - 1) * sizeof(Island));
                    --n;

                    merged_any = true;
                    break;  // restart scanning from i=0
                }
            }
            if (merged_any) break;
        }
    } while (merged_any);

    *n_islands = n;
    return list;
}


static uint8_t * _sort_palette;
static int     * _sort_counts;
static int cmp_order_desc(const void *pa, const void *pb) {
    int ia = *(const int*)pa;
    int ib = *(const int*)pb;
    // сравниваем по убыванию counts[ palette[idx] ]
    return _sort_counts[_sort_palette[ib]]
        - _sort_counts[_sort_palette[ia]];
}


/**
 * собирает палитру/гистограмму с учётом RGB332, сортирует цвета по частоте, сохраняет в структуре фон и «второй» цвет. */
static void classify_island(Island *isl, const uint8_t *quant) {
    // collect palette and counts
    uint8_t *palette; int palette_n; int *counts;
    collect_palette_hist(quant, g.padded_w, isl, &palette, &palette_n, &counts);
    // total pixels
    int total = isl->count * BS * BS;
    isl->total_pixels = total;
    // sort by frequency
    int order[256];
    for (int i = 0; i < palette_n; ++i) order[i] = i;
    _sort_palette = palette;
    _sort_counts = counts;
    qsort(order, palette_n, sizeof(int), cmp_order_desc);
    // set bg and fg
    isl->bg_color = palette[order[0]];
    isl->fg_color = (palette_n > 1) ? palette[order[1]] : isl->bg_color;
    // cleanup
    free(palette);
    free(counts);
}



static void dump_islands_png(const char *fname,
                             const uint8_t *quant,
                             const uint8_t *block_color,
                             const Island *islands,
                             int island_n)
{
    int W  = g.w;
    int H  = g.h;
    int pw = g.padded_w;

    // 1) Фон — белый
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { perror("dump_islands_png"); return; }
    memset(rgb, 255, (size_t)W * H * 3);

    // 2) Для каждого острова
    for (int i = 0; i < island_n; ++i) {
        const Island *isl = &islands[i];

        // a) Сначала отрисовываем UNIFORM-блоки в стиле fill
        for (int bi = 0; bi < isl->count; ++bi) {
            int b = isl->blocks[bi];
            uint8_t c = block_color[b];
            if (c == MIXED) continue;  // только uniform

            int by = b / g.block_cols, bx = b % g.block_cols;
            int x0 = bx * BS, y0 = by * BS;
            int w_blk = MIN(BS, W - x0), h_blk = MIN(BS, H - y0);

            // Распаковываем цвет из RGB332
            uint8_t r3 = (c >> 5) & 0x7;
            uint8_t g3 = (c >> 2) & 0x7;
            uint8_t b2 =  c        & 0x3;
            uint8_t r8 = expand3(r3);
            uint8_t g8 = expand3(g3);
            uint8_t b8 = expand2(b2);

            // 1) Заливка
            for (int dy = 0; dy < h_blk; ++dy) {
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = ((size_t)(y0+dy) * W + (x0+dx)) * 3;
                    rgb[p+0] = r8;
                    rgb[p+1] = g8;
                    rgb[p+2] = b8;
                }
            }
            // 2) Рамки
            // верхняя и левая — белые
            for (int dx = 0; dx < w_blk; ++dx) {
                size_t p = (size_t)y0 * W + (x0+dx);
                rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
            }
            for (int dy = 0; dy < h_blk; ++dy) {
                size_t p = (size_t)(y0+dy) * W + x0;
                rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
            }
            // нижняя и правая — чёрные
            for (int dx = 0; dx < w_blk; ++dx) {
                size_t p = (size_t)(y0+h_blk-1) * W + (x0+dx);
                rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
            }
            for (int dy = 0; dy < h_blk; ++dy) {
                size_t p = (size_t)(y0+dy) * W + (x0+w_blk-1);
                rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
            }
            // диагональ ↘ — чёрная
            {
                int dx=0, dy=0, err=0;
                while (dx < w_blk && dy < h_blk) {
                    size_t p = (size_t)(y0+dy)*W + (x0+dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                    err += h_blk;
                    if (err >= w_blk) { err -= w_blk; ++dy; }
                    ++dx;
                }
            }
            // диагональ ↙ — белая
            {
                int dx=0, dy=0, err=0;
                while (dx < w_blk && dy < h_blk) {
                    size_t p = (size_t)(y0+dy)*W + (x0 + w_blk - 1 - dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                    err += h_blk;
                    if (err >= w_blk) { err -= w_blk; ++dy; }
                    ++dx;
                }
            }
        }

        // b) Затем отрисовываем MIXED-блоки, раскрашивая пиксели в bg/fg
        for (int bi = 0; bi < isl->count; ++bi) {
            int b = isl->blocks[bi];
            /* if (block_color[b] != MIXED) continue; */

            int by = b / g.block_cols, bx = b % g.block_cols;
            int x0 = bx * BS, y0 = by * BS;
            int w_blk = MIN(BS, W - x0), h_blk = MIN(BS, H - y0);

            for (int dy = 0; dy < h_blk; ++dy) {
                for (int dx = 0; dx < w_blk; ++dx) {
                    uint8_t q = quant[(y0+dy)*pw + (x0+dx)];
                    uint8_t use = (q == isl->bg_color)
                        ? isl->bg_color
                        : isl->fg_color;
                    uint8_t r3 = (use >> 5) & 0x7;
                    uint8_t g3 = (use >> 2) & 0x7;
                    uint8_t b2 =  use       & 0x3;
                    size_t p = ((size_t)(y0+dy) * W + (x0+dx)) * 3;
                    rgb[p+0] = expand3(r3);
                    rgb[p+1] = expand3(g3);
                    rgb[p+2] = expand2(b2);
                }
            }
        }

        // c) В конце bounding-box острова
        {
            uint8_t rr, gg, bb;
            pick_island_color(i, &rr, &gg, &bb);
            int minx = MAX(0, MIN(W-1, isl->minx));
            int miny = MAX(0, MIN(H-1, isl->miny));
            int maxx = MAX(0, MIN(W-1, isl->maxx));
            int maxy = MAX(0, MIN(H-1, isl->maxy));
            // top/bottom
            for (int x = minx; x <= maxx; ++x) {
                size_t p1 = (size_t)miny*W + x;
                size_t p2 = (size_t)maxy*W + x;
                rgb[p1*3+0] = rr; rgb[p1*3+1] = gg; rgb[p1*3+2] = bb;
                rgb[p2*3+0] = rr; rgb[p2*3+1] = gg; rgb[p2*3+2] = bb;
            }
            // left/right
            for (int y = miny; y <= maxy; ++y) {
                size_t p1 = (size_t)y*W + minx;
                size_t p2 = (size_t)y*W + maxx;
                rgb[p1*3+0] = rr; rgb[p1*3+1] = gg; rgb[p1*3+2] = bb;
                rgb[p2*3+0] = rr; rgb[p2*3+1] = gg; rgb[p2*3+2] = bb;
            }
        }
    }

    // 3) Сохранение и очистка
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
}


// Compile‐time helper to dump each island’s shape and uniform‐block highlights
static void debug_dump_islands(int slot,
                               const uint8_t *quant,
                               const uint8_t *block_color,
                               const Island *islands,
                               int island_n)
{
    char fn[64];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn),
             "dbg_islands_%d_%ld.png",
             slot, ts.tv_sec);
    // Draw each island over the quantized background, highlighting uniform blocks
    dump_islands_png(fn,
                     quant,
                     block_color,
                     islands,
                     island_n);
}


static void *worker_thread(void *arg)
{
    /* uintptr_t wid = (uintptr_t)arg; */
    /* size_t px_cnt = (size_t)g.padded_w * g.padded_h; */
    /* char dbg_fname[128]; */

    for (;;) {
        for (uint32_t i = 0; i < g.slots; ++i) {
            FrameSlot *s = &g.slot[i];
            enum slot_state exp = RAW_READY;
            if (!atomic_compare_exchange_strong(&s->st, &exp, IN_PROGRESS))
                continue;

            /* debug_dump_raw(i, s->raw, g.w, g.h); */

            // === Квантование RGB332 + анализ uniform-блоков 32×32
            // Принимает: s->raw - RGBA8888
            // Возвращает:
            //    s->quant,    // RGB332-byte per px
            //    s->color,    // блоки 32×32: uniform-цвет или MIXED (0xFF)
            // Ничего не выделяет, что следовало бы освободить.
            quantize_and_analyze(s);

            debug_dump_quant(i, s->quant, g.padded_w);

            debug_dump_filled(i, s);

/*             // === Найти "острова" MIXED-блоков */
/*             // Принимает: s->color - блоки 8×8: uniform-цвет или MIXED (0xFF) */
/*             // Возвращает: */
/*             //    *islands,    // Массив "островов" */
/*             //    island_n,    // Количество найденных "островов" */
/*             // Выделяет: */
/*             //    *islands */
/*             int island_n; */
/*             Island *islands = detect_mixed_islands(s->color, g.block_rows, g.block_cols, &island_n); */

/*             for (int i = 0; i < island_n; ++i) { */
/*                 compute_island_bbox(&islands[i]); */
/*             } */

/*             /\* islands = merge_islands(islands, &island_n); *\/ */
/*             for (int i = 0; i < island_n; ++i) { */
/*                 classify_island(&islands[i], s->quant); */
/*             } */


/*             //  Debug dump islands */
/*             debug_dump_islands(i, */
/*                                s->quant,     // квантованный кадр */
/*                                s->color,     // цветовые метки блоков */
/*                                islands, */
/*                                island_n); */


            /* // Теперь очищаем все blocks и сам массив islands */
            /* for (int isl_i = 0; isl_i < island_n; ++isl_i) { */
            /*     free(islands[isl_i].blocks); */
            /* } */
            /* free(islands); */

            // Ставим слот в готовое состояние
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
int main(int argc, char **argv) {
    // 1. Парсинг аргументов --slots=N
    g.slots = DEFAULT_SLOTS;
    for (int i = 1; i < argc; ++i) {
        if (strncmp(argv[i], "--slots=", 8) == 0) {
            int v = atoi(argv[i] + 8);
            if (v < 1 || v > MAX_SLOTS) {
                fprintf(stderr, "slots 1..%d\n", MAX_SLOTS);
                return 1;
            }
            g.slots = v;
        } else {
            fprintf(stderr, "unknown option %s\n", argv[i]);
            return 1;
        }
    }

    // 2. Определяем число рабочих потоков
    int cores = sysconf(_SC_NPROCESSORS_ONLN);
    g.workers = (cores > 4) ? cores - 3 : 1;

    // 3. Инициализация X11 и многосегментный SHM
    init_x11();  // внутри создаются g.slots XImage+SHM

    // 4. Вычисление размеров блоков 32×32 и паддинга
    g.block_cols  = (g.w + BS - 1) / BS;
    g.block_rows  = (g.h + BS - 1) / BS;
    g.block_count = g.block_cols * g.block_rows;
    g.padded_w    = g.block_cols * BS;  // теперь ≥ w и кратно 32
    g.padded_h    = g.block_rows * BS;  // теперь ≥ h и кратно 32

    fprintf(stdout,
            "[init] screen %dx%d, padded %dx%d, blocks %dx%d (%d total)\n",
            g.w, g.h, g.padded_w, g.padded_h,
            g.block_cols, g.block_rows, g.block_count);

    // 5. Аллокация больших буферов для quant + color
    allocate_bigmem();

    // 6. Запуск потоков: захват, обработка, сериализация
    pthread_t cap_tid, ser_tid;
    pthread_t *w_tids = calloc(g.workers, sizeof(pthread_t));

    pthread_create(&cap_tid, NULL, capture_thread, NULL);
    pthread_create(&ser_tid, NULL, serializer_thread, NULL);

    for (uint32_t i = 0; i < g.workers; ++i) {
        pthread_create(&w_tids[i], NULL, worker_thread, (void*)(uintptr_t)i);
    }

    // 7. Ждём потоков (на самом деле capture_thread бесконечен)
    pthread_join(cap_tid, NULL);
    pthread_join(ser_tid, NULL);
    for (uint32_t i = 0; i < g.workers; ++i) {
        pthread_join(w_tids[i], NULL);
    }

    return 0;
}
