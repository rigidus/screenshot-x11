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


/* ---- tiny helper: write RGB888 buffer to PNG ------------------------- */
static void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb)
{
    FILE *fp = fopen(fname, "wb");
    if (!fp) { perror("png dump"); return; }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop   inf = png_create_info_struct(png);
    if (!png || !inf || setjmp(png_jmpbuf(png))) { fclose(fp); return; }

    png_init_io(png, fp);
    png_set_IHDR(png, inf, W, H, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, inf);

    for (int y = 0; y < H; ++y)
        png_write_row(png, (png_bytep)(rgb + y * W * 3));

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &inf);
    fclose(fp);
}

/* --- helper: expand 5-bit to 8-bit (for dumping quant) ---------------- */
static inline uint8_t expand5(uint16_t v){ return (v<<3)|(v>>2); }



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
    uint16_t *quant;
    uint16_t *color;
} FrameSlot;

typedef struct {
    /* размеры экрана */
    int w,h;
    /* дерево */
    RectTopo *topo;
    uint32_t  node_cnt, leaf_cnt, topo_cap;
    /* слоты */
    uint32_t  slots, workers;
    FrameSlot slot[MAX_SLOTS];
    uint8_t  *bigmem;
    size_t    bigmem_sz;
    /* X11 */
    Display  *dpy; Window root; XImage *ximg; XShmSegmentInfo shm;
} Ctx;  static Ctx g;

/* ---------- объявления -------------------------------------------------- */
static void build_topology(int,int);
static void allocate_bigmem(void);
static void init_x11(void);
static void *capture_thread(void*);
static void *worker_thread(void*);
static void *serializer_thread(void*);
static void  write_qimg(const FrameSlot*, const char*);

/* ═════════════════════════ topo-builder ═══════════════════════════════════ */

/*
 * Добавляет узел-прямоугольник в уже заранее
 * выделенный g.topo[0 .. g.topo_cap-1].
 * При переполнении сразу завершаем программу,
 * чтобы не повредить память.
 */
static uint32_t
push_rect(uint16_t x0, uint16_t y0,
          uint16_t x1, uint16_t y1)
{
    if (g.node_cnt >= g.topo_cap)            /* страхуемся */
        die("topology overflow");            /* malloc too small */

    uint32_t idx = g.node_cnt++;             /* позиция для записи */

    g.topo[idx] = (RectTopo){
        .x0 = x0, .y0 = y0,
        .x1 = x1, .y1 = y1,
        .L  = UINT32_MAX,                    /* пока нет потомков   */
        .R  = UINT32_MAX,
        .P  = UINT32_MAX
    };
    return idx;                              /* вернуть индекс узла */
}

/*
 * split_recursive(0) делит корень, а потом рекурсивно — каждый получившийся
 * под-прямоугольник, пока его ширина и высота не станут ≤ 16 px
 * (константа MIN_LEAF).
 * Финал: в g.topo лежит полный массив узлов, упорядоченный
 * "по месту создания", а в счётчиках — их количество.
 */
static void split_recursive(uint32_t idx)
{
    /* работаем через индекс, а не через сохранённый указатель */
    uint16_t w = g.topo[idx].x1 - g.topo[idx].x0;
    uint16_t h = g.topo[idx].y1 - g.topo[idx].y0;

    if (w <= MIN_LEAF && h <= MIN_LEAF) {  /* лист - базовый случай */
        g.leaf_cnt++;
        return;
    }

    uint32_t l, r;
    if (w >= h) {                          /* режем по X */
        uint16_t m = g.topo[idx].x0 + w / 2;
        l = push_rect(g.topo[idx].x0, g.topo[idx].y0, m,           g.topo[idx].y1);
        r = push_rect(m,              g.topo[idx].y0, g.topo[idx].x1, g.topo[idx].y1);
    } else {                               /* режем по Y */
        uint16_t m = g.topo[idx].y0 + h / 2;
        l = push_rect(g.topo[idx].x0, g.topo[idx].y0, g.topo[idx].x1, m);
        r = push_rect(g.topo[idx].x0, m,              g.topo[idx].x1, g.topo[idx].y1);
    }

    /* записываем индексы родителя и детей */
    g.topo[l].P = g.topo[r].P = idx;
    g.topo[idx].L = l;
    g.topo[idx].R = r;


    split_recursive(l); // рекурсивно делим левый
    split_recursive(r); // и правый прямоугольники
}

/* ближайшая степень-2 ≥ n  (n>0) */
static inline uint32_t pow2_ge(uint32_t n)
{
    return 1u << (32 - __builtin_clz(n - 1));
}

/*
 * build_topology() — это функция, которая строит "скелет"
 * бинарного дерева Rect (topology) для текущего размера экрана.
 * Дерево понадобится всем остальным этапам конвейера: воркеры будут
 * отмечать цвета листьев, а сериализатор — сжимать одинаковые соседние узлы
 * merge_uniform_nodes() поднимается снизу вверх и схлопывает одинаковые цвета
 * write_qimg() сериализует дерево и листовые данные.
 */
static void build_topology(int w,int h)
{
    fprintf(stdout,"[init] topo: w=%d h=%d\n", w, h);

    /* Дерево полно: каждый нелист имеет ровно 2 потомка. */
    /*  Если известно L — количество листьев, то общее число узлов */
    /*  N = 2 × L – 1.  */
    /*  leaves = pow2_ge(ceil(W/16)) × pow2_ge(ceil(H/16)) */
    /* Сколько листьев? */
    /*  Алгоритм режет, пока обе стороны блока > 16 px. */
    /* Для 1854 × 1107: */
    /*  | шаг      | В/16 | следующий pow2 | Н/16 | следующий pow2 | */
    /*  | значения | 118  | 128            | 70   | 128            | */

    uint32_t blkX = (w + MIN_LEAF - 1) / MIN_LEAF;   /* ceil(w/16)  */
    uint32_t blkY = (h + MIN_LEAF - 1) / MIN_LEAF;   /* ceil(h/16)  */

    uint32_t leaves = pow2_ge(blkX) * pow2_ge(blkY);
    uint32_t nodes  = leaves * 2 - 1;

    /* g.node_cnt — сколько узлов уже записано; */
    /* g.leaf_cnt — сколько из них являются листьями (конечные прямоугольники); */
    /* g.topo_cap — текущий размер выделенного массива g.topo */

    g.node_cnt = g.leaf_cnt = 0; // обнуляем счётчики
    g.topo_cap = leaves*2 - 1;
    g.topo = malloc(g.topo_cap * sizeof(RectTopo));

    g.topo_cap = nodes;
    g.topo     = malloc(nodes * sizeof(RectTopo));
    if (!g.topo) die("malloc topo");

    push_rect(0, 0, (uint16_t)w, (uint16_t)h);

    /* split_recursive(0) делит корень, а потом рекурсивно — каждый получившийся */
    /* под-прямоугольник, пока его ширина и высота не станут ≤ 16 px */
    /* (константа MIN_LEAF). */
    split_recursive(0);

    /* Финал: в g.topo лежит полный массив узлов, упорядоченный */
    /* "по месту создания", а в счётчиках — их количество.  */
    fprintf(stdout, "[init] topo: nodes=%u leaves=%u\n",
            g.node_cnt, g.leaf_cnt); /* проверка: должны совпасть
                                        с nodes/leaves, т.е. всё влезло */
}

/* ═════════════════════════ память и X11 ═══════════════════════════════════ */
static void allocate_bigmem(void)
{
    /* если нужны строгие 64-байтовые выравнивания для AVX-инструкций, добавьте
       uintptr_t align = (uintptr_t)base & 63; if(align) base += 64 - align;
       перед расчётом raw.  */

    // Для каждого слота:
    //  base
    //  ├── raw   : [0 .. raw_sz)
    //  ├── quant : [raw_sz .. raw_sz+q_sz)
    //  └── color : [raw_sz+q_sz .. raw_sz+q_sz+col_sz)
    size_t raw_sz   = (size_t)g.w*g.h*4;                    // ARGB8888
    size_t q_sz     = (size_t)g.w*g.h*2;                    // RGB555
    size_t col_sz   = (size_t)g.node_cnt*sizeof(uint16_t);  // цвет каждого узла
    size_t per_slot = raw_sz+q_sz+col_sz;                   // один slot
    g.bigmem_sz     = per_slot*g.slots;                     // все slots

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
    for ( uint32_t i=0; i<g.slots; ++i ) {
        uint8_t *base = g.bigmem + i*per_slot;
        g.slot[i] = (FrameSlot) {
            .st    = FREE,                          // начальное состояние
            .raw   = base,                          // начало — сырой BGRA
            .quant = (uint16_t*)(base+raw_sz),      // следом — квант-буфер
            .color = (uint16_t*)(base+raw_sz+q_sz)  // в конце — массив цветов
        };
    }

    fprintf(stdout,"[init] bigmem %.1f MiB (per slot %.1f MiB)\n",
            g.bigmem_sz/1048576.0, per_slot/1048576.0);
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

    /* Создаём XImage в разделяемой памяти */
    g.ximg=XShmCreateImage(g.dpy,DefaultVisual(g.dpy,scr),
                           DefaultDepth(g.dpy,scr),ZPixmap,
                           NULL,&g.shm,g.w,g.h);
    if(!g.ximg) die("XShmCreateImage");
    g.shm.shmid = shmget(IPC_PRIVATE,
                         g.ximg->bytes_per_line*g.ximg->height,
                         IPC_CREAT|0600);
    if(g.shm.shmid<0) die("shmget");

    g.shm.shmaddr = g.ximg->data = shmat(g.shm.shmid,NULL,0);
    if ( g.shm.shmaddr==(char*)-1 )  die("shmat");
    if ( !XShmAttach(g.dpy,&g.shm) ) die("XShmAttach");
    XSync(g.dpy,False);

    fprintf(stdout,"[init] X11 %dx%d\n",g.w,g.h);
}


/* ═════════════════════════ SIMD RGB888→RGB555 ═════════════════════════════ */
static void quantize_rgb555(const uint8_t *src, uint16_t *dst, size_t px)
{
#ifdef __SSE2__
    size_t i=0;
    for(; i+4<=px; i+=4,src+=16,dst+=4){
        /* BGRA BGRA …  (little-endian).  Нужно отбросить альфу, оставить
           ровно 5 младших бит каждой компоненты.                           */
        __m128i v   = _mm_loadu_si128((const __m128i*)src);
        __m128i mask= _mm_set1_epi32(0x1F);                   /* 0001 1111b */

        __m128i r5 = _mm_and_si128(_mm_srli_epi32(v,16+3), mask);  /* R >>3 */
        __m128i g5 = _mm_and_si128(_mm_srli_epi32(v, 8+3), mask);  /* G >>3 */
        __m128i b5 = _mm_and_si128(_mm_srli_epi32(v,    3), mask); /* B >>3 */

        __m128i rgb = _mm_or_si128(_mm_slli_epi32(r5,10),
                                   _mm_or_si128(_mm_slli_epi32(g5,5), b5)); /* 0RRRRR0GGGGG0BBBBB */

        /* упаковываем 32-бит → 16-бит, сохраняем 4 пикселя */
        _mm_storel_epi64((__m128i*)dst, _mm_packs_epi32(rgb,_mm_setzero_si128()));
    }
#else
    size_t i=0;
#endif
    for(; i<px; ++i,src+=4)
        *dst++ = (uint16_t)(((src[2]>>3)<<10)|((src[1]>>3)<<5)|(src[0]>>3));
}


/* ═════════════════════════ анализ листа ═══════════════════════════════════ */
static uint16_t leaf_uniform_color(const uint16_t *base,int stride,const RectTopo *r)
{
    const uint16_t c0=base[r->y0*stride + r->x0];
    __m128i vc0=_mm_set1_epi16(c0);

    for(uint16_t y=r->y0; y<r->y1; ++y){
        const uint16_t *row = base + y*stride + r->x0;
        uint16_t w=r->x1-r->x0, vcnt=w/8;
        for(uint16_t v=0; v<vcnt; ++v){
            if(_mm_movemask_epi8(
                   _mm_cmpeq_epi16(_mm_loadu_si128((const __m128i*)row),vc0))
               !=0xFFFF)
                return COLOR_MIXED;
            row+=8;
        }
        for(uint16_t x=r->x0+vcnt*8; x<r->x1; ++x)
            if(*row++!=c0) return COLOR_MIXED;
    }
    return c0;
}
static void analyse_leaves(const uint16_t *quant,uint16_t *color)
{
    int stride=g.w;
    for(uint32_t i=0;i<g.node_cnt;++i){
        const RectTopo *r=&g.topo[i];
        color[i]=(r->L==UINT32_MAX)?
            leaf_uniform_color(quant,stride,r):COLOR_MIXED;
    }
}
static void merge_uniform_nodes(uint16_t *color)
{
    size_t merged = 0;
    for ( int32_t i = (int32_t)g.node_cnt-1; i>=0; --i ) {
        const RectTopo *r = &g.topo[i];
        if ( r->L == UINT32_MAX ) continue;
        uint16_t cL = color[r->L], cR=color[r->R];

        if (cL != COLOR_MIXED && cL == cR) {
            color[i] = cL;
            ++merged;
        } else {
            color[i] = COLOR_MIXED;
        }
    }
    fprintf(stdout, "[debug] merged nodes = %zu\n", merged);
}


/* ---------- запись QIMG (исправлен magic) ------------------------------- */
static void write_qimg(const FrameSlot *s, const char *fn)
{
    FILE *fp=fopen(fn,"wb"); if(!fp){ perror("fopen qimg"); return; }

    uint32_t hdr[3] = {MAGIC_QIMG,
        (uint16_t)g.w | ((uint32_t)g.h<<16),
        g.node_cnt};
    fwrite(hdr,4,3,fp);

    fwrite(s->color,2,g.node_cnt,fp);

    for(uint32_t i=0;i<g.node_cnt;++i){
        const RectTopo *r=&g.topo[i];
        if(r->L!=UINT32_MAX) continue;
        if(s->color[i]==COLOR_MIXED){
            for(uint16_t y=r->y0;y<r->y1;++y){
                const uint16_t *row=s->quant + y*g.w + r->x0;
                fwrite(row,2,r->x1-r->x0,fp);
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
        if ( !XShmGetImage(g.dpy, g.root, g.ximg, 0, 0, AllPlanes) ) {
            fprintf(stderr, "XShmGetImage failed\n");
            continue;
        }
        memcpy(s->raw, g.ximg->data, (size_t)g.w * g.h * 4);
        s->t_start = now_ns();
        atomic_store_explicit(&s->st, RAW_READY, memory_order_release);
        idx = (idx + 1) % g.slots;
    }
    return NULL;
}

/* ================= worker_thread ======================================= */
static void *worker_thread(void *arg)
{

#ifdef HAVE_LIBNUMA
    uintptr_t wid = (uintptr_t)arg;
#else
    (void)arg; /* не используется */
#endif

    /* NUMA-pinning  ------------------------------------------------------ */
#ifdef HAVE_LIBNUMA
    int nodes = numa_num_configured_nodes();
    if (nodes > 0) {
        int node = wid % nodes;
        numa_run_on_node(node);

        cpu_set_t m;  CPU_ZERO(&m);
        int cpus_per = sysconf(_SC_NPROCESSORS_ONLN) / (nodes ?: 1);
        if (cpus_per < 1) cpus_per = 1;
        int cid = (wid % cpus_per) + node * cpus_per;
        CPU_SET(cid, &m);
        pthread_setaffinity_np(pthread_self(), sizeof m, &m);
    }
#endif

    const size_t px_cnt = (size_t)g.w * g.h;

    for (;;) {
        for (uint32_t i = 0; i < g.slots; ++i) {
            FrameSlot *s = &g.slot[i];

            enum slot_state exp = RAW_READY;
            if (!atomic_compare_exchange_strong(&s->st, &exp, IN_PROGRESS)) {
                /* не удалось захватить slot */
                continue;
            }

            /* -------- 1. quantize BGRA8888 → RGB555 -------------------- */
            quantize_rgb555(s->raw, s->quant, px_cnt);

            /* -------- DEBUG-PNG  (пишем один раз на слот) -------------- */
            if (getenv("XCAP_DEBUG") || i == 0) {
                /* dump RAW → PNG (один раз) */
                static _Atomic int dumped_raw[MAX_SLOTS]  = {0};
                if (!dumped_raw[i]) {
                    uint8_t *rgb = malloc(px_cnt * 3);
                    const uint8_t *src = s->raw;
                    for (size_t p = 0; p < px_cnt; ++p, src += 4) {
                        rgb[p*3+0] = src[2];   /* R */
                        rgb[p*3+1] = src[1];   /* G */
                        rgb[p*3+2] = src[0];   /* B */
                    }
                    char fn[64];  snprintf(fn,sizeof fn,"dbg_RAW_%u.png", i);
                    dump_png_rgb(fn, g.w, g.h, rgb);
                    free(rgb);
                    dumped_raw[i] = 1;
                }

                /* dump QUANT → PNG (один раз) */
                static _Atomic int dumped_q[MAX_SLOTS] = {0};
                if (!dumped_q[i]) {
                    uint8_t *rgb = malloc(px_cnt * 3);
                    const uint16_t *q = s->quant;
                    for (size_t p = 0; p < px_cnt; ++p, ++q) {
                        uint16_t v = *q;
                        rgb[p*3+0] = expand5((v >> 10) & 0x1F);
                        rgb[p*3+1] = expand5((v >>  5) & 0x1F);
                        rgb[p*3+2] = expand5( v        & 0x1F);
                    }
                    char fn[64];  snprintf(fn,sizeof fn,"dbg_QUANT_%u.png", i);
                    dump_png_rgb(fn, g.w, g.h, rgb);
                    free(rgb);
                    dumped_q[i] = 1;
                }
            }

            /* -------- 2. analyse leaves + merge ------------------------ */
            analyse_leaves(s->quant, s->color);
            merge_uniform_nodes(s->color);


            /* -------- DEBUG: PNG с пометкой однородных блоков ---------- */
            if (getenv("XCAP_DEBUG") || i == 0) {
                static _Atomic int dumped_m[MAX_SLOTS] = {0};
                if (!dumped_m[i]) {
                    /* сначала скопируем QUANT→RGB888 в буфер */
                    uint8_t *rgb = malloc(px_cnt * 3);
                    const uint16_t *q = s->quant;
                    for (size_t p = 0; p < px_cnt; ++p, ++q) {
                        uint16_t v = *q;
                        rgb[p*3+0] = expand5((v>>10)&0x1F);
                        rgb[p*3+1] = expand5((v>>5 )&0x1F);
                        rgb[p*3+2] = expand5( v      &0x1F);
                    }

                    /* обойдём все листья и нарисуем диагонали там,
                       где color != COLOR_MIXED                              */
                    for (uint32_t n = 0; n < g.node_cnt; ++n) {
                        if (g.topo[n].L != UINT32_MAX)   continue;      /* не лист */
                        if (s->color[n] == COLOR_MIXED)  continue;

                        /* /\* новый фильтр «родитель уже однороден и совпадает» *\/ */
                        /* uint32_t p = g.topo[n].P; */
                        /* while (p != UINT32_MAX && */
                        /*     s->color[p] != COLOR_MIXED && */
                        /*     s->color[p] == s->color[n]) */
                        /*     continue;                                   /\* лист спрятан *\/ */


                        /* поиск однородного предка */
                        uint32_t p = g.topo[n].P;
                        while (p != UINT32_MAX &&
                               s->color[p] != COLOR_MIXED &&
                               s->color[p] == s->color[n])
                            p = g.topo[p].P;                    /* поднимаемся выше */

                        if (p != UINT32_MAX)                    /* нашли «поглотителя» */
                            continue;                           /* → этот лист НЕ рисуем */


                        const RectTopo *r = &g.topo[n];
                        uint32_t w = r->x1 - r->x0;
                        uint32_t h = r->y1 - r->y0;

                        /* диагональ ↘   (Bresenham y = (h/w)*x) */
                        for (uint32_t x = 0, y = 0, err = 0; x < w && y < h; ++x) {
                            size_t p = ((r->y0 + y) * g.w + (r->x0 + x)) * 3;
                            rgb[p] = rgb[p+1] = rgb[p+2] = 255;          /* белый  */
                            err += h;
                            if (err >= (uint)w) { err -= w; ++y; }
                        }

                        /* диагональ ↙   (из TR в BL) */
                        for (uint32_t x = 0, y = 0, err = 0; x < w && y < h; ++x) {
                            size_t p = ((r->y0 + y) * g.w + (r->x1 - 1 - x)) * 3;
                            rgb[p] = rgb[p+1] = rgb[p+2] = 0;            /* чёрный */
                            err += h;
                            if (err >= (uint)w) { err -= w; ++y; }
                        }

                        /* верхняя горизонталь (y = y0) – белая */
                        for (uint32_t x = r->x0; x < r->x1; ++x){
                            size_t p = (r->y0 * g.w + x) * 3;
                            rgb[p] = rgb[p+1] = rgb[p+2] = 255;
                        }
                        /* нижняя горизонталь (y = y1-1) – чёрная */
                        for (uint32_t x = r->x0; x < r->x1; ++x){
                            size_t p = ((r->y1 - 1) * g.w + x) * 3;
                            rgb[p] = rgb[p+1] = rgb[p+2] = 0;
                        }
                        /* левая вертикаль (x = x0) – белая */
                        for (uint32_t y = r->y0; y < r->y1; ++y){
                            size_t p = (y * g.w + r->x0) * 3;
                            rgb[p] = rgb[p+1] = rgb[p+2] = 255;
                        }
                        /* правая вертикаль (x = x1-1) – чёрная */
                        for (uint32_t y = r->y0; y < r->y1; ++y){
                            size_t p = (y * g.w + (r->x1 - 1)) * 3;
                            rgb[p] = rgb[p+1] = rgb[p+2] = 0;
                        }

                    }

                    /* ─── подчеркнём объединённые узлы (не листья) ─── */
                    for (uint32_t n = 0; n < g.node_cnt; ++n) {
                        if (g.topo[n].L == UINT32_MAX) continue;           /* лист пропускаем */
                        if (s->color[n] == COLOR_MIXED) continue;          /* не объединён   */

                        /* условие «оба потомка совпадают с родителем» уже гарантировано
                           функцией merge_uniform_nodes(), но уточним явно */
                        if (   s->color[g.topo[n].L] != s->color[n]
                               || s->color[g.topo[n].R] != s->color[n])
                            continue;

                        const RectTopo *r = &g.topo[n];

                        /* оранжевая рамка (#FFA500) вокруг объединённого прямоугольника */
                        for (uint32_t x = r->x0; x < r->x1; ++x){          /* верх */
                            size_t p = (r->y0 * g.w + x) * 3;
                            rgb[p]=255; rgb[p+1]=165; rgb[p+2]=0;
                        }
                        for (uint32_t x = r->x0; x < r->x1; ++x){          /* низ */
                            size_t p = ((r->y1-1) * g.w + x) * 3;
                            rgb[p]=255; rgb[p+1]=165; rgb[p+2]=0;
                        }
                        for (uint32_t y = r->y0; y < r->y1; ++y){          /* левый */
                            size_t p = (y * g.w + r->x0) * 3;
                            rgb[p]=255; rgb[p+1]=165; rgb[p+2]=0;
                        }
                        for (uint32_t y = r->y0; y < r->y1; ++y){          /* правый */
                            size_t p = (y * g.w + (r->x1-1)) * 3;
                            rgb[p]=255; rgb[p+1]=165; rgb[p+2]=0;
                        }

                        /* ─── диагональ ↗   (BL → TR) ─── */
                        uint32_t x0 = r->x0, y0 = r->y0, x1 = r->x1, y1 = r->y1;

                        /* ─── диагональ ↗   (BL → TR) ─── */
                        uint32_t w = x1 - x0;
                        uint32_t h = y1 - y0;

                        /* Брезенхэм: идём по X, поднимаемся по Y */
                        for (uint32_t x = 0, y = h - 1, err = 0;
                             x < w && y < h; ++x)                           /* y – unsigned, <h всегда true */
                        {
                            size_t p = ((y0 + y) * g.w + (x0 + x)) * 3;
                            rgb[p]   = 255;      /* R */
                            rgb[p+1] = 165;      /* G */
                            rgb[p+2] = 0;        /* B */

                            /* классический error-accumulator */
                            if (h >= w) {                /* "крутая" диагональ */
                                err += w;
                                if (err >= (uint)h) { err -= h; if (y) --y; }
                            } else {                     /* "пологая" */
                                err += h;
                                if (err >= (uint)w) { err -= w; if (y) --y; }
                            }
                        }
                    }

                    char fn[64];  snprintf(fn,sizeof fn,"dbg_MARKED_%u.png", i);
                    dump_png_rgb(fn, g.w, g.h, rgb);
                    free(rgb);
                    dumped_m[i] = 1;
                }
            }

            /* -------- 3. done ------------------------------------------ */
            atomic_store_explicit(&s->st, QUANT_DONE, memory_order_release);
        }
        sched_yield();    /* отдаём квант, чтобы не крутить CPU всухую */
    }
    return NULL;          /* формально */
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
    build_topology(g.w,g.h);
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
