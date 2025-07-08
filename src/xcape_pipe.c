/*
 * xcap_pipe.c
 * ----------------------------------------------------------------------------
 * Захватывает скриншоты X11 через XShm, квантизирует их до RGB555,
 * рекурсивно делит изображение на прямоугольники  ≥ 32×32 px, для каждого
 * ...
 *
 */

/* ---------------------- компиляция ---------------------------------------
 * CC     := gcc
 * CFLAGS := -O3 -std=c11 -Wall -Wextra -pedantic
 * LIBPNG := -lpng
 * XLIBS  := -lX11 -lXext          # нужны только для capture, можно оставить
 *
 * SRC    := src
 * BIN    := xcape_pipe
 *
 * CXX ?= g++
 *
 * xcape_pipe: src/xcape_pipe.c
 *      xcape_pipe: src/xcape_pipe.c
 $(CC) -O3 -march=native -std=c17 -Wall -Wextra -pthread $<  -lXext -lX11 -lnuma -lpng -o $@
 *
 * dbg:
 *      ./xcape_pipe --slots=1
 *
 * ------------------------------------------------------------------------ */

#define _GNU_SOURCE
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <png.h>
#include <numa.h>
#include <emmintrin.h>
#include <immintrin.h>
#include <xmmintrin.h>
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
#include <limits.h>


/* ---------- константы и вспомогалки ------------------------------------ */
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define MIXED         0xFF             // маркер «mixed»-блока
#define BS 32
#define MAX_NEIGHBOR_GAP 32  // максимально допустимый gap по X между буквами
#define MAX_HEIGHT_DIFF      6     // максимально допустимая разница высот
#define MAX_VERTICAL_OFFSET  (BS/2) // отклонение по центру

#define TEMPLATE_SIZE 16            // template width/height in pixels
#define TEMPLATE_PIXELS (TEMPLATE_SIZE * TEMPLATE_SIZE)
#define TEMPLATE_BYTES ((TEMPLATE_PIXELS + 7) / 8)  // bytes per binary mask
#define MATCH_THRESHOLD 0.75f       // required match score for recognition
#define MAX_REGIONS 1024            // maximum number of detected regions
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
    uint8_t  *raw;         // указатель на сырое изображение RGB в XShm-буфере
    uint8_t  *quant;       // RGB332 (один байт на пиксель)
    uint8_t  *bg;          // фоновой цвет каждого блока (1 байт на блок)
    uint8_t  *fg;          // цвет текста - второй по распространенности в каждом блоке (1 байт на блок)
    uint8_t  *mask;        // битовая маска: 1 бит на пиксель блока – (BS*BS)/8 байт на блок (должна быть выровнена)
    int       block_count; // число блоков (всего по всему изображению)
} FrameSlot;


typedef struct {
    /* исходное разрешение экрана */
    int w, h;

    /* разрешение с паддингом до кратности 32×32 */
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


typedef struct {
    uint16_t by, bx;  // block indices
    uint8_t  dy, dx;  // pixel offsets inside block
} Pixel;


typedef struct Region {
    int minx, miny, maxx, maxy;  // bounding box in global pixels
    int count;                   // number of pixels in region
    Pixel *pixels;               // array of Pixel
    struct Region *neighbor;     // next region on the right, or NULL
} Region;


// Новая структура для групп строк
typedef struct {
    Region **regions;  // указатели на регионы в этой строке
    int      count;    // сколько регионов в строке
    // computed bbox:
    int minx, miny, maxx, maxy;
} Line;


// Template for a character
typedef struct {
    char ch;
    uint8_t mask[TEMPLATE_BYTES];  // bitmask of TEMPLATE_SIZE×TEMPLATE_SIZE
} Template;


/* extern int      n_templates; */
/* extern Template templates[];      // defined elsewhere, size n_templates */
Template templates[] = {
    // {'A', { /* 16×16 битовая маска для 'A' */ }},
    // {'B', { /* ... */ }},
    // и т.д. для всех нужных символов
};
int n_templates = sizeof(templates) / sizeof(templates[0]);


/* ---------- объявления -------------------------------------------------- */
static void allocate_bigmem(void);
static void init_x11(void);
static void *capture_thread(void*);
static void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb);
static void *worker_thread(void*);
static void *serializer_thread(void*);


/* ═════════════════════════ память и X11 ═══════════════════════════════════ */

#define ALIGN 128

// округление вверх до кратности ALIGN
static inline size_t align_up(size_t x) {
    return (x + (ALIGN - 1)) & ~(ALIGN - 1);
}


static void allocate_bigmem(void)
{
    /* если нужны строгие 64-байтовые выравнивания для AVX-инструкций, добавьте
       uintptr_t align = (uintptr_t)base & 63; if(align) base += 64 - align;
       перед расчётом raw.  */

    // размер квант-буфера (1 байт на пиксель (RBG332)) с паддингом
    size_t q_sz              = g.padded_w * g.padded_h * 1;
    // размер буфера цветов фона всех блоков (1 байт на цвет)
    size_t bg_sz        = g.block_count;
    // размер буфера цветов текста всех блоков (1 байт на цвет)
    size_t fg_sz        = g.block_count;
    // размер буфера битовой маски одного блока.
    // блоки всегда кратны 8 пикселям
    size_t single_mask  = (BS * BS) / 8;
    // размер буфера битовых масок всех блоков
    size_t mask_sz      = g.block_count * single_mask;   // все блоки

    // смещения с учётом выравниваний
    size_t offset_quant = 0;
    size_t offset_bg    = align_up(offset_quant + q_sz);
    size_t offset_fg    = align_up(offset_bg + bg_sz);
    size_t offset_mask  = align_up(offset_fg + fg_sz);

    // размер памяти одного слота
    size_t per_slot     = align_up(offset_mask + mask_sz);

    // размер памяти под все слоты
    g.bigmem_sz         = per_slot * g.slots;

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
            .raw   = (uint8_t*)g.ximg[i]->data, // raw данные лежат в shm-образе
            .quant = base + offset_quant,       // квант-буфер
            .bg    = base + offset_bg,          // bg
            .fg    = base + offset_fg,          // fg
            .mask  = base + offset_mask,        // mask
            .block_count = g.block_count
        };
        fprintf(stderr,
                "[slot %u] quant=%p bg=%p fg=%p mask=%p\n",
                i, g.slot[i].quant, g.slot[i].bg,
                g.slot[i].fg,    g.slot[i].mask
            );
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
        }
    }
}


/**
 * Detect connected regions of 1-bits in mixed-block masks.
 * @param mask           bitmasks concatenated: block_count * mask_bytes
 * @param block_color    per-block flag: MIXED if mixed, 0 if uniform
 * @param block_rows     number of blocks vertically
 * @param block_cols     number of blocks horizontally
 * @param BS             block size (pixels)
 * @param out_region_n   output: number of regions found
 * @return dynamically allocated array of Region; caller must free each pixels and the array
 */
Region* detect_regions(const uint8_t *mask,
                       const uint8_t *block_color,
                       int block_rows, int block_cols,
                       int *out_region_n) {
    int mask_bytes = (BS * BS + 7) / 8;
    int total_pix  = block_rows * block_cols * BS * BS;
    bool *visited  = calloc(total_pix, sizeof(bool));
    Pixel *queue   = malloc(total_pix * sizeof(Pixel));

    Region *regions = NULL;
    int region_n = 0, region_cap = 0;

    for (int by = 0; by < block_rows; ++by) for (int bx = 0; bx < block_cols; ++bx) {
            int bidx = by * block_cols + bx;
            if (block_color[bidx] != MIXED) continue;
            const uint8_t *mask_block = mask + bidx * mask_bytes;
            for (int dy = 0; dy < BS; ++dy) for (int dx = 0; dx < BS; ++dx) {
                    int bit   = dy * BS + dx;
                    if (!(mask_block[bit>>3] & (1 << (bit&7)))) continue;
                    int local = (bidx * BS + dy) * BS + dx;
                    if (visited[local]) continue;

                    // start BFS
                    int qh = 0, qt = 0;
                    queue[qt++] = (Pixel){by,(uint16_t)bx,(uint8_t)dy,(uint8_t)dx};
                    visited[local] = true;

                    if (region_n == region_cap) {
                        region_cap = region_cap ? region_cap*2 : 8;
                        regions = realloc(regions, region_cap * sizeof(Region));
                    }
                    Region *reg = &regions[region_n++];
                    reg->count  = 0;
                    reg->pixels = NULL;
                    int gx0 = bx*BS + dx;
                    int gy0 = by*BS + dy;
                    reg->minx = reg->maxx = gx0;
                    reg->miny = reg->maxy = gy0;

                    const int d4[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
                    while (qh < qt) {
                        Pixel p = queue[qh++];
                        // append pixel
                        reg->pixels = realloc(reg->pixels, (reg->count+1) * sizeof(Pixel));
                        reg->pixels[reg->count++] = p;
                        // update bbox
                        int gx = p.bx*BS + p.dx;
                        int gy = p.by*BS + p.dy;
                        if (gx < reg->minx) reg->minx = gx;
                        if (gx > reg->maxx) reg->maxx = gx;
                        if (gy < reg->miny) reg->miny = gy;
                        if (gy > reg->maxy) reg->maxy = gy;
                        // neighbors
                        for (int k = 0; k < 4; ++k) {
                            int ndy = p.dy + d4[k][0];
                            int ndx = p.dx + d4[k][1];
                            int nby = p.by, nbx = p.bx;
                            if      (ndy < 0)      { nby = p.by - 1; ndy = BS - 1; }
                            else if (ndy >= BS)    { nby = p.by + 1; ndy = 0;    }
                            if      (ndx < 0)      { nbx = p.bx - 1; ndx = BS - 1; }
                            else if (ndx >= BS)    { nbx = p.bx + 1; ndx = 0;    }
                            if (nby < 0 || nby >= block_rows || nbx < 0 || nbx >= block_cols)
                                continue;
                            int nbidx = nby * block_cols + nbx;
                            if (block_color[nbidx] != MIXED) continue;
                            const uint8_t *nb_mask = mask + nbidx * mask_bytes;
                            int nbit = ndy * BS + ndx;
                            if (!(nb_mask[nbit>>3] & (1 << (nbit&7)))) continue;
                            int nl = (nbidx * BS + ndy) * BS + ndx;
                            if (visited[nl]) continue;
                            visited[nl] = true;
                            queue[qt++] = (Pixel){(uint16_t)nby,(uint16_t)nbx,(uint8_t)ndy,(uint8_t)ndx};
                        }
                    }
                }
        }

    free(queue);
    free(visited);
    *out_region_n = region_n;
    return regions;
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


// расширение RGB332 → [0..255]
static inline void expand_rgb332(uint8_t q, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = (q >> 4) & 0x3; *r = (*r<<6)|(*r<<4)|(*r<<2)|*r;
    *g = (q >> 2) & 0x3; *g = (*g<<6)|(*g<<4)|(*g<<2)|*g;
    *b = (q     ) & 0x3; *b = (*b<<6)|(*b<<4)|(*b<<2)|*b;
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
    /* const int pw  = g.padded_w; */
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



/**
 * Debug: dump regions with black pixels and colored bounding boxes
 */
void debug_dump_regions(int slot_idx,
                        const Region *regions,
                        int region_n) {
    /* extern int g.w, g.h, BS; */
    uint8_t *rgb = malloc((size_t)g.w * g.h * 3);
    memset(rgb, 255, (size_t)g.w * g.h * 3);
    // draw black pixels for regions
    for (int r = 0; r < region_n; ++r) {
        for (int i = 0; i < regions[r].count; ++i) {
            Pixel p = regions[r].pixels[i];
            int x = p.bx*BS + p.dx;
            int y = p.by*BS + p.dy;
            size_t off = ((size_t)y * g.w + x) * 3;
            rgb[off+0] = rgb[off+1] = rgb[off+2] = 0;
        }
    }
    // draw colored bounding boxes
    srand(slot_idx);
    for (int r = 0; r < region_n; ++r) {
        uint8_t rr = rand() & 255;
        uint8_t gg = rand() & 255;
        uint8_t bb = rand() & 255;
        int minx = regions[r].minx;
        int maxx = regions[r].maxx;
        int miny = regions[r].miny;
        int maxy = regions[r].maxy;
        for (int x = minx; x <= maxx; ++x) {
            size_t t = ((size_t)miny * g.w + x) * 3;
            size_t b = ((size_t)maxy * g.w + x) * 3;
            rgb[t+0]=rr; rgb[t+1]=gg; rgb[t+2]=bb;
            rgb[b+0]=rr; rgb[b+1]=gg; rgb[b+2]=bb;
        }
        for (int y = miny; y <= maxy; ++y) {
            size_t l = ((size_t)y * g.w + minx) * 3;
            size_t rgt = ((size_t)y * g.w + maxx) * 3;
            rgb[l+0]=rr; rgb[l+1]=gg; rgb[l+2]=bb;
            rgb[rgt+0]=rr; rgb[rgt+1]=gg; rgb[rgt+2]=bb;
        }
    }
    char fn[64]; struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "regions_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_png_rgb(fn, g.w, g.h, rgb);
    free(rgb);
}



// Render region into 16×16 sample mask
void render_region(const Region *reg, uint8_t *sample) {
    memset(sample, 0, TEMPLATE_BYTES);
    for (int i = 0; i < reg->count; ++i) {
        Pixel p = reg->pixels[i];
        int gx = p.bx*BS + p.dx;
        int gy = p.by*BS + p.dy;
        int rx = gx - reg->minx;
        int ry = gy - reg->miny;
        if (rx < TEMPLATE_SIZE && ry < TEMPLATE_SIZE) {
            int bit = ry * TEMPLATE_SIZE + rx;
            sample[bit>>3] |= 1 << (bit & 7);
        }
    }
}

// Popcount of 8-bit mask
static inline int popcount8(uint8_t v) { return __builtin_popcount(v); }

// Recognize single region by template matching
char recognize_region(const Region *reg) {
    uint8_t sample[TEMPLATE_BYTES];
    render_region(reg, sample);
    float best_score = 0.0f;
    char  best_ch    = '?';

    for (int t = 0; t < n_templates; ++t) {
        int match = 0;
        for (int i = 0; i < TEMPLATE_BYTES; ++i) {
            uint8_t xnor = ~(sample[i] ^ templates[t].mask[i]);
            match += popcount8(xnor);
        }
        float score = (float)match / TEMPLATE_PIXELS;
        if (score > best_score) {
            best_score = score;
            best_ch    = templates[t].ch;
        }
    }
    return best_score >= MATCH_THRESHOLD ? best_ch : '?';
}


// Debug: dump recognized letters
void debug_recognize(int slot_idx,
                     Region *regions, int region_n) {
    for (int r = 0; r < region_n; ++r) {
        char c = recognize_region(&regions[r]);
        printf("slot %d, region %d: '%c'\n", slot_idx, r, c);
    }
}




// Функция группировки Region в строки по вертикали
static int cmp_region_miny(const void *a, const void *b) {
    const Region *r1 = *(Region**)a;
    const Region *r2 = *(Region**)b;
    return r1->miny - r2->miny;
}

static int cmp_region_minx(const void *a, const void *b) {
    const Region *r1 = *(Region**)a;
    const Region *r2 = *(Region**)b;
    return r1->minx - r2->minx;
}

void group_regions(Region *regions, int region_n,
                   Line **out_lines, int *out_line_n) {
    if (region_n == 0) { *out_lines = NULL; *out_line_n = 0; return; }

    // Собираем массив указателей и сортируем по miny
    Region **arr = malloc(region_n * sizeof(Region*));
    for (int i = 0; i < region_n; i++) arr[i] = &regions[i];
    qsort(arr, region_n, sizeof(Region*), cmp_region_miny);

    // Средняя высота региона
    float avg_h = 0;
    for (int i = 0; i < region_n; i++)
        avg_h += (arr[i]->maxy - arr[i]->miny + 1);
    avg_h /= region_n;

    const float GAP = avg_h * 0.6f;

    Line *lines = NULL;
    int   line_n = 0;

    // Кластеризуем жадно
    for (int i = 0; i < region_n; i++) {
        Region *r = arr[i];
        if (line_n == 0) {
            lines = realloc(lines, sizeof(Line) * (line_n+1));
            lines[line_n].regions = malloc(sizeof(Region*));
            lines[line_n].regions[0] = r;
            lines[line_n].count = 1;
            line_n++;
        } else {
            Line *L = &lines[line_n-1];
            // вычисляем текущий maxy в последней строке
            int maxy = L->miny = L->maxy = L->regions[0]->miny;
            for (int j = 0; j < L->count; j++) {
                if (L->regions[j]->maxy > maxy) maxy = L->regions[j]->maxy;
                if (j==0) L->miny = L->regions[j]->miny;
                else if (L->regions[j]->miny < L->miny) L->miny = L->regions[j]->miny;
            }
            // если r лежит по-вертикали близко — добавляем в эту строку
            if (r->miny <= maxy + GAP) {
                L->regions = realloc(L->regions, sizeof(Region*)*(L->count+1));
                L->regions[L->count++] = r;
            } else {
                // новая строка
                lines = realloc(lines, sizeof(Line) * (line_n+1));
                lines[line_n].regions = malloc(sizeof(Region*));
                lines[line_n].regions[0] = r;
                lines[line_n].count = 1;
                line_n++;
            }
        }
    }

    // внутри каждой строки сортируем по X и вычисляем bbox
    for (int i = 0; i < line_n; i++) {
        Line *L = &lines[i];
        qsort(L->regions, L->count, sizeof(Region*), cmp_region_minx);
        // init bbox
        L->minx = L->regions[0]->minx;
        L->maxx = L->regions[0]->maxx;
        L->miny = L->regions[0]->miny;
        L->maxy = L->regions[0]->maxy;
        for (int j = 1; j < L->count; j++) {
            Region *r = L->regions[j];
            if (r->minx < L->minx) L->minx = r->minx;
            if (r->miny < L->miny) L->miny = r->miny;
            if (r->maxx > L->maxx) L->maxx = r->maxx;
            if (r->maxy > L->maxy) L->maxy = r->maxy;
        }
    }

    free(arr);
    *out_lines  = lines;
    *out_line_n = line_n;
}


// Отладочный дамп строк: рисуем bbox каждой строки цветной рамкой
void debug_dump_lines(int slot_idx, Line *lines, int line_n) {
    uint8_t *rgb = malloc((size_t)g.w * g.h * 3);
    memset(rgb, 255, (size_t)g.w * g.h * 3);

    srand(slot_idx);
    for (int i = 0; i < line_n; i++) {
        Line *L = &lines[i];
        uint8_t rr = rand() & 255;
        uint8_t gg = rand() & 255;
        uint8_t bb = rand() & 255;
        // по периметру bbox
        for (int x = L->minx; x <= L->maxx; x++) {
            size_t t = ((size_t)L->miny * g.w + x)*3;
            size_t b = ((size_t)L->maxy * g.w + x)*3;
            rgb[t+0]=rr; rgb[t+1]=gg; rgb[t+2]=bb;
            rgb[b+0]=rr; rgb[b+1]=gg; rgb[b+2]=bb;
        }
        for (int y = L->miny; y <= L->maxy; y++) {
            size_t l = ((size_t)y * g.w + L->minx)*3;
            size_t r = ((size_t)y * g.w + L->maxx)*3;
            rgb[l+0]=rr; rgb[l+1]=gg; rgb[l+2]=bb;
            rgb[r+0]=rr; rgb[r+1]=gg; rgb[r+2]=bb;
        }
    }

    char fn[64];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "lines_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_png_rgb(fn, g.w, g.h, rgb);
    free(rgb);
}

// =============================================================================
/* // Для каждого маленького региона (<32×32) найдем «соседа» справа */
/* static void set_region_neighbors(Region *regions, int region_n) { */
/*     // Сбросим все указатели */
/*     for (int i = 0; i < region_n; i++) { */
/*         regions[i].neighbor = NULL; */
/*     } */
/*     // Для каждого маленького региона найдём «соседа» справа */
/*     for (int i = 0; i < region_n; i++) { */
/*         Region *ri = &regions[i]; */
/*         int wi = ri->maxx - ri->minx + 1; */
/*         int hi = ri->maxy - ri->miny + 1; */
/*         // интересуют только регионы меньше 32×32 */
/*         if (wi >= 32 || hi >= 32) continue; */

/*         int best_dx = INT_MAX; */
/*         int best_j  = -1; */
/*         int cy_i    = (ri->miny + ri->maxy) / 2; */

/*         for (int j = 0; j < region_n; j++) { */
/*             if (i == j) continue; */
/*             Region *rj = &regions[j]; */
/*             int wj = rj->maxx - rj->minx + 1; */
/*             int hj = rj->maxy - rj->miny + 1; */
/*             if (wj >= 32 || hj >= 32) continue; */

/*             int dx = rj->minx - ri->maxx; */
/*             // справа, в пределах MAX_NEIGHBOR_GAP и лучше по dx */
/*             if (dx <= 0 || dx > MAX_NEIGHBOR_GAP || dx >= best_dx) continue; */

/*             // похож по размерам */
/*             if (abs(wj - wi) > 4) continue; */

/*             // похож по вертикальному положению */
/*             int cy_j = (rj->miny + rj->maxy) / 2; */
/*             if (abs(cy_j - cy_i) > BS/4) continue; */

/*             best_dx = dx; */
/*             best_j  = j; */
/*         } */

/*         if (best_j >= 0) { */
/*             ri->neighbor = &regions[best_j]; */
/*         } */
/*     } */
/* } */

static void set_region_neighbors(Region *regions, int region_n) {
    // Сброс всех ссылок
    for (int i = 0; i < region_n; i++) {
        regions[i].neighbor = NULL;
    }

    for (int i = 0; i < region_n; i++) {
        Region *ri = &regions[i];
        int wi = ri->maxx - ri->minx + 1;
        int hi = ri->maxy - ri->miny + 1;
        // только «маленькие» регионы
        if (wi >= BS || hi >= BS) continue;

        int best_dx = INT_MAX, best_j = -1;
        int cy_i    = (ri->miny + ri->maxy) / 2;

        for (int j = 0; j < region_n; j++) {
            if (i == j) continue;
            Region *rj = &regions[j];
            int wj = rj->maxx - rj->minx + 1;
            int hj = rj->maxy - rj->miny + 1;
            if (wj >= BS || hj >= BS) continue;

            int dx = rj->minx - ri->maxx;
            // справа, не дальше, чем MAX_NEIGHBOR_GAP, и лучше, чем предыдущий
            if (dx <= 0 || dx > MAX_NEIGHBOR_GAP || dx >= best_dx) continue;

            // **правильно**: сравниваем по высоте, а не по ширине
            if (abs(hj - hi) > MAX_HEIGHT_DIFF) continue;

            int cy_j = (rj->miny + rj->maxy) / 2;
            if (abs(cy_j - cy_i) > MAX_VERTICAL_OFFSET) continue;

            best_dx = dx;
            best_j  = j;
        }

        if (best_j >= 0) {
            ri->neighbor = &regions[best_j];
        }
    }
}


// Debug: рисуем сначала рамки всех маленьких регионов, затем по «цепочкам» общие bbox строк
static void debug_dump_chains(int slot_idx, Region *regions, int region_n) {
    uint8_t *rgb = malloc((size_t)g.w * g.h * 3);
    memset(rgb, 255, (size_t)g.w * g.h * 3);

    // 1) Рисуем чёрные рамки вокруг каждого маленького региона
    for (int i = 0; i < region_n; i++) {
        Region *r = &regions[i];
        int w = r->maxx - r->minx + 1;
        int h = r->maxy - r->miny + 1;
        if (w < 32 && h < 32) {
            for (int x = r->minx; x <= r->maxx; x++) {
                size_t t = ((size_t)r->miny * g.w + x)*3;
                size_t b = ((size_t)r->maxy * g.w + x)*3;
                rgb[t+0] = rgb[t+1] = rgb[t+2] = 0;
                rgb[b+0] = rgb[b+1] = rgb[b+2] = 0;
            }
            for (int y = r->miny; y <= r->maxy; y++) {
                size_t l = ((size_t)y * g.w + r->minx)*3;
                size_t rr= ((size_t)y * g.w + r->maxx)*3;
                rgb[l+0] = rgb[l+1] = rgb[l+2] = 0;
                rgb[rr+0]= rgb[rr+1]= rgb[rr+2]= 0;
            }
        }
    }

    // 2) Помечаем все регионы, на которые кто-то ссылается как на neighbor
    bool *is_target = calloc(region_n, sizeof(bool));
    for (int i = 0; i < region_n; i++) {
        Region *nbr = regions[i].neighbor;
        if (nbr) {
            int idx = (int)(nbr - regions);
            if (idx >= 0 && idx < region_n) {
                is_target[idx] = true;
            }
        }
    }

        // 3) Для каждого «головы» цепочки (не является target, но имеет neighbor) —
        //    проходим по цепочке и рисуем красную рамку вокруг общего bbox
    srand(slot_idx + 123);
    for (int i = 0; i < region_n; i++) {
        Region *r0 = &regions[i];
        if (!is_target[i] && r0->neighbor) {
            int minx = r0->minx, miny = r0->miny;
            int maxx = r0->maxx, maxy = r0->maxy;
            // обходим цепочку
            for (Region *cur = r0; cur; cur = cur->neighbor) {
                if (cur->minx < minx) minx = cur->minx;
                if (cur->miny < miny) miny = cur->miny;
                if (cur->maxx > maxx) maxx = cur->maxx;
                if (cur->maxy > maxy) maxy = cur->maxy;
            }

            // рисуем красную рамку для всей строки
            for (int x = minx; x <= maxx; x++) {
                size_t t = ((size_t)miny * g.w + x)*3;
                size_t b = ((size_t)maxy * g.w + x)*3;
                rgb[t+0] = 255; rgb[t+1] =   0; rgb[t+2] =   0;
                rgb[b+0] = 255; rgb[b+1] =   0; rgb[b+2] =   0;
            }
            for (int y = miny; y <= maxy; y++) {
                size_t l = ((size_t)y * g.w + minx)*3;
                size_t rr= ((size_t)y * g.w + maxx)*3;
                rgb[l+0] = 255; rgb[l+1] =   0; rgb[l+2] =   0;
                rgb[rr+0]= 255; rgb[rr+1]=   0; rgb[rr+2]=   0;
            }
        }
    }
    free(is_target);

    // 4) Сохраняем
    char fn[64]; struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "chains_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_png_rgb(fn, g.w, g.h, rgb);
    free(rgb);
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

            /* debug_dump_quant(i, s->quant, g.padded_w); */

            /* debug_dump_filled(i, s); */

            // build per-block color flags: MIXED if fg xor bg != 0
            int bc = g.block_rows * g.block_cols;
            uint8_t *block_color = malloc(bc);
            for (int i = 0; i < bc; ++i) {
                block_color[i] = (s->fg[i] ^ s->bg[i]) ? MIXED : 0;
            }
            int region_n;
            Region *regions = detect_regions(s->mask, block_color,
                                             g.block_rows, g.block_cols,
                                             &region_n);

            // ——— Отфильтруем слишком большие регионы (>32×32) ———
            int small_n = 0;
            Region *small = malloc(region_n * sizeof(Region));
            for (int r = 0; r < region_n; r++) {
                int w = regions[r].maxx - regions[r].minx + 1;
                int h = regions[r].maxy - regions[r].miny + 1;
                if (w <= 32 && h <= 32) {
                    // оставляем этот регион
                    small[small_n++] = regions[r];
                } else {
                    // слишком большой — сразу освобождаем его пиксели
                    free(regions[r].pixels);
                }
            }
            free(regions);
            regions    = small;
            region_n   = small_n;
            // ——————————————————————————————————————————————————————————

            debug_dump_regions(i, regions, region_n);

            /* for (int r = 0; r < region_n; ++r) free(regions[r].pixels); */
            /* free(regions); */
            /* free(block_color); */

            debug_recognize(i, regions, region_n);


            // связываем «соседей» и рисуем цепочки
            set_region_neighbors(regions, region_n);
            debug_dump_chains(i, regions, region_n);


            for (int r = 0; r < region_n; ++r)
                free(regions[r].pixels);
            free(regions);
            free(block_color);


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
                /* write_qimg(s,fn); */
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
    g.padded_w    = g.block_cols * BS;  //  ≥ w и кратно 32
    g.padded_h    = g.block_rows * BS;  //  ≥ h и кратно 32

    fprintf(stdout,
            "[init] screen %dx%d, padded %dx%d, blocks %dx%d (%d total)\n",
            g.w, g.h, g.padded_w, g.padded_h,
            g.block_cols, g.block_rows, g.block_cols * g.block_rows);

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
