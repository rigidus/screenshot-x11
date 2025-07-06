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
    uint8_t  *quant;   // RGB222 + старший transparency bit
    uint8_t  *color;   // массив цветов блоков uniform-цвет блока 8×8 или MIXED
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


/// Описывает один «остров» смешанных блоков
typedef struct {
    int *blocks;   // массив индексов блоков (row*cols + col)
    int  count;
    int  cap;
} Island;


/* ---------- объявления -------------------------------------------------- */
static void allocate_bigmem(void);
static void init_x11(void);
static void *capture_thread(void*);
static void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb);
static void dump_islands_png(const char *fname, const uint8_t *quant, const Island *islands, int island_n);
void decompress_island_to_png(const char *fname,
                              const Island *isl,
                              int padded_w,
                              uint8_t palette[2],
                              const uint8_t *mask);
static void dump_all_islands_png(const char *fname,
                                 const uint8_t *quant,
                                 int padded_w,
                                 const Island *islands,
                                 int island_n);
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
    size_t q_sz     = (size_t)g.w*g.h;                    // RGB322
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


/* ═════════════════════════ SIMD RGB888→RGB222 ═════════════════════════════ */

#define BLOCK_SIZE    16
#define MIXED         0xFF


// Расширение 2-бит → 8-бит
static inline uint8_t expand2(uint8_t v) {
    return (uint8_t)((v << 6) | (v << 4) | (v << 2) | v);
}

// Расширение 3-бит → 8-бит
static inline uint8_t expand3(uint8_t v) {
    return (uint8_t)((v << 5) | (v << 2) | (v >> 1));
}

// CPUID‐утилита для AVX2
static inline bool have_avx2(void) {
    uint32_t a, b, c, d;
    __asm__ volatile("cpuid"
                     : "=a"(a), "=b"(b), "=c"(c), "=d"(d)
                     : "a"(7), "c"(0)
        );
    return (b & (1<<5)) != 0;  // AVX2 в EBX[5]
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
    for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
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

static void quantize_and_analyze(
    const uint8_t *src0,      // ARGB8888, stride = w*4
    uint8_t       *quant,     // [padded_h][padded_w]
    uint8_t       *block_color, // [block_count]
    int            w,
    int            h
    ) {
    const int pw = g.padded_w;
    const int ph = g.padded_h;
    const int bc = g.block_cols;
    const int br = g.block_rows;

    // Однократный детект AVX2
    static bool inited = false;
    static bool use256;
    if (!inited) {
        use256 = cpu_has_avx2();
        inited = true;
    }

    // Инициализация состояний блоков
    for (int i = 0; i < br*bc; ++i) {
        block_color[i] = MIXED;
    }

    // 1) Квантование RGB888 → RGB332 (AVX2/SSE2/scalar)
    for (int y = 0; y < ph; ++y) {
        const uint8_t *row_src = src0 + (size_t)y * w * 4;
        uint8_t       *row_q   = quant + (size_t)y * pw;
        int x = 0;

        if (use256) {
            __m256i mR = _mm256_set1_epi32(0xE0);
            __m256i mB = _mm256_set1_epi32(0xC0);
            int lim8 = (w/8)*8;
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
        // SSE2
        {
            __m128i mR4 = _mm_set1_epi32(0xE0);
            __m128i mB4 = _mm_set1_epi32(0xC0);
            int lim4 = (w/4)*4;
            for (; x < lim4; x += 4) {
                __m128i pix = _mm_loadu_si128((__m128i*)(row_src + x*4));
                __m128i r   = _mm_and_si128(_mm_srli_epi32(pix,16), mR4);
                __m128i g2  = _mm_and_si128(_mm_srli_epi32(pix, 8), mR4);
                __m128i b   = _mm_and_si128(pix,               mB4);
                __m128i rg  = _mm_or_si128(r, _mm_srli_epi32(g2,3));
                __m128i rgb = _mm_or_si128(rg, _mm_srli_epi32(b,6));
                __m128i p16 = _mm_packus_epi32(rgb, _mm_setzero_si128());
                __m128i p8  = _mm_packus_epi16(p16, _mm_setzero_si128());
                *(uint32_t*)(row_q + x) = _mm_cvtsi128_si32(p8);
            }
        }
        // Scalar tail + padding
        for (; x < w; ++x) {
            uint8_t R = row_src[x*4+2], G = row_src[x*4+1], B = row_src[x*4+0];
            uint8_t R3 = R >> 5, G3 = G >> 5, B2 = B >> 6;
            row_q[x] = (uint8_t)((R3<<5)|(G3<<2)|B2);
        }
        for (; x < pw; ++x) {
            row_q[x] = 0;
        }
    }

    _mm_sfence();  // дождаться всех store

    // 2) Анализ однородности блоков 32×32
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {
            int idx = by*bc + bx;
            int sy  = by * BLOCK_SIZE;
            int sx  = bx * BLOCK_SIZE;

            bool uniform;
            if (use256) {
                uniform = block_uniform_avx2(quant, pw, sy, sx);
            } else {
                // scalar fallback
                uint8_t base = quant[sy*pw + sx];
                if (base == 0) {
                    uniform = true;
                } else {
                    uniform = true;
                    for (int dy = 0; dy < BLOCK_SIZE && uniform; ++dy) {
                        for (int dx = 0; dx < BLOCK_SIZE; ++dx) {
                            uint8_t q = quant[(sy+dy)*pw + (sx+dx)];
                            if (q != 0 && q != base) {
                                uniform = false;
                                break;
                            }
                        }
                    }
                }
            }
            block_color[idx] = uniform
                ? quant[sy*pw + sx]  // base
                : MIXED;
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
 * dump_filled_blocks_png
 *
 * Рисует отладочное изображение, где каждый блок 8×8:
 * - если block_color == MIXED, внутри блока отображаются
 *   квантованные пиксели (RGB222 → RGB888);
 * - иначе блок заливается своим цветом, на нём:
 *     • левая и верхняя границы — белые,
 *     • правая и нижняя — чёрные,
 *     • диагональ ↘ (из TL в BR) — чёрная (алгоритм Брезенхема),
 *     • диагональ ↙ (из TR в BL) — белая.
 */


static void dump_filled_blocks_png(const char *fname,
                                   const uint8_t *quant,
                                   const uint8_t *block_color)
{
    int W = g.w, H = g.h;
    int pw = g.padded_w;
    int bc = g.block_cols, br = g.block_rows;

    // Буфер RGB888 для сохранения
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { perror("malloc"); return; }
    // Фон — белый
    memset(rgb, 255, (size_t)W * H * 3);

    // Проход по блокам 8×8
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {
            int idx = by * bc + bx;
            uint8_t c = block_color[idx];
            int x0 = bx * BLOCK_SIZE;
            int y0 = by * BLOCK_SIZE;
            int w_blk = MIN(BLOCK_SIZE, W - x0);
            int h_blk = MIN(BLOCK_SIZE, H - y0);

            if (c == MIXED) {
                // MIXED: рисуем квантованные пиксели
                for (int dy = 0; dy < h_blk; ++dy) {
                    for (int dx = 0; dx < w_blk; ++dx) {
                        uint8_t q = quant[(y0 + dy) * pw + (x0 + dx)];
                        uint8_t r2 = (q >> 4) & 3;
                        uint8_t g2 = (q >> 2) & 3;
                        uint8_t b2 =  q       & 3;
                        uint8_t r8 = expand3(r2);
                        uint8_t g8 = expand3(g2);
                        uint8_t b8 = expand2(b2);
                        size_t p = (size_t)(y0 + dy) * W + (x0 + dx);
                        rgb[p*3 + 0] = r8;
                        rgb[p*3 + 1] = g8;
                        rgb[p*3 + 2] = b8;
                    }
                }
            } else {
                // UNIFORM: заливка блока своим цветом
                uint8_t r2 = (c >> 4) & 3;
                uint8_t g2 = (c >> 2) & 3;
                uint8_t b2 =  c       & 3;
                uint8_t r8 = expand3(r2);
                uint8_t g8 = expand3(g2);
                uint8_t b8 = expand2(b2);
                // заполнить
                for (int dy = 0; dy < h_blk; ++dy) {
                    for (int dx = 0; dx < w_blk; ++dx) {
                        size_t p = (size_t)(y0 + dy) * W + (x0 + dx);
                        rgb[p*3 + 0] = r8;
                        rgb[p*3 + 1] = g8;
                        rgb[p*3 + 2] = b8;
                    }
                }
                // рамки
                // верхняя и левая — белые
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = (size_t)y0 * W + (x0 + dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                }
                for (int dy = 0; dy < h_blk; ++dy) {
                    size_t p = (size_t)(y0 + dy) * W + x0;
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                }
                // нижняя и правая — чёрные
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = (size_t)(y0 + h_blk - 1) * W + (x0 + dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                }
                for (int dy = 0; dy < h_blk; ++dy) {
                    size_t p = (size_t)(y0 + dy) * W + (x0 + w_blk - 1);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                }
                // диагональ ↘ (чёрная) — Bresenham
                {
                    int dx = 0, dy = 0, err = 0;
                    while (dx < w_blk && dy < h_blk) {
                        size_t p = (size_t)(y0 + dy) * W + (x0 + dx);
                        rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                        err += h_blk;
                        if (err >= w_blk) {
                            err -= w_blk;
                            ++dy;
                        }
                        ++dx;
                    }
                }
                // диагональ ↙ (из TR в BL, белая) — Bresenham
                {
                    int dx = 0, dy = 0, err = 0;
                    while (dx < w_blk && dy < h_blk) {
                        size_t p = (size_t)(y0 + dy) * W + (x0 + w_blk - 1 - dx);
                        rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                        err += h_blk;
                        if (err >= w_blk) {
                            err -= w_blk;
                            ++dy;
                        }
                        ++dx;
                    }
                }
            }
        }
    }

    // Сохраняем PNG
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
}





/// Находит все связанные «острова» блоков, у которых block_color==MIXED.
/// Возвращает массив Island[ *out_n ], сам массив и поля need to be freed by caller.
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


/* signature: palette и counts выделяются внутри и возвращаются через указатели
 * асимптотика O(BLOCK_SIZE² × number_of_blocks_in_island)
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
        int y0 = br*BLOCK_SIZE, x0 = bc*BLOCK_SIZE;
        // Перебор всех пикселей в блоке
        for (int dy=0; dy<BLOCK_SIZE; ++dy) {
            for (int dx=0; dx<BLOCK_SIZE; ++dx) {
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





/// Генерация «полупрозрачного» цвета из индекса
static void pick_island_color(int idx, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Простой хеш: меняем компоненты по-разному
    *r = (uint8_t)( 50 + (idx * 37) % 156 );
    *g = (uint8_t)(100 + (idx * 59) % 156 );
    *b = (uint8_t)(150 + (idx * 83) %  76 );
}

/// Рисует огибающую рамку заданного острова (сплошной прямоугольник)
static void draw_island_bbox(uint8_t *rgb, int W, int H,
                             const Island *isl, int padded_w,
                             uint8_t rr, uint8_t gg, uint8_t bb)
{
    // найдем min/max по x и y среди блоков
    int minx = W, miny = H, maxx = 0, maxy = 0;
    for (int i = 0; i < isl->count; ++i) {
        int b = isl->blocks[i];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int x0 = bx * BLOCK_SIZE, y0 = by * BLOCK_SIZE;
        minx = x0 < minx ? x0 : minx;
        miny = y0 < miny ? y0 : miny;
        maxx = (x0 + BLOCK_SIZE - 1) > maxx ? x0 + BLOCK_SIZE - 1 : maxx;
        maxy = (y0 + BLOCK_SIZE - 1) > maxy ? y0 + BLOCK_SIZE - 1 : maxy;
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
        int x0 = bx * BLOCK_SIZE, y0 = by * BLOCK_SIZE;
        for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
            int y = y0 + dy; if (y >= H) break;
            for (int dx = 0; dx < BLOCK_SIZE; ++dx) {
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


/// Debug: отрисовать острова поверх квантованного фона
static void dump_islands_png(const char *fname,
                             const uint8_t *quant,
                             const Island *islands,
                             int island_n)
{
    int W = g.w, H = g.h, pw = g.padded_w;
    // 1) подготовить фон из quant → RGB888
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { perror("malloc"); return; }
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint8_t q = quant[y * pw + x];
            uint8_t r2 = (q >> 5) & 7;
            uint8_t g2 = (q >> 2) & 7;
            uint8_t b2 =  q       & 3;
            rgb[((size_t)y*W + x)*3 + 0] = expand3(r2);
            rgb[((size_t)y*W + x)*3 + 1] = expand3(g2);
            rgb[((size_t)y*W + x)*3 + 2] = expand2(b2);
        }
    }
    // 2) для каждого острова — pick цвет и рисуем
    for (int i = 0; i < island_n; ++i) {
        uint8_t rr, gg, bb;
        pick_island_color(i, &rr, &gg, &bb);
        draw_island_bbox(rgb, W, H, &islands[i], pw, rr, gg, bb);
    }
    // 3) записать PNG
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
}


// расширение RGB222 → [0..255]
static inline void expand_rgb222(uint8_t q, uint8_t *r, uint8_t *g, uint8_t *b) {
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
    expand_rgb222(c0,&r0,&g0,&b0);
    for (int i = 0; i < pn; ++i) if (i != best0) {
            uint8_t ri,gi,bi;
            expand_rgb222(pl[i], &ri, &gi, &bi);
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
        expand_rgb222(q,&ri,&gi,&bi);
        double d0 = (r0 - ri)*(r0 - ri) + (g0 - gi)*(g0 - gi) + (b0 - bi)*(b0 - bi);
        // раскладка для c1
        uint8_t r1,g1,b1; expand_rgb222(c1,&r1,&g1,&b1);
        double d1 = (r1 - ri)*(r1 - ri) + (g1 - gi)*(g1 - gi) + (b1 - bi)*(b1 - bi);
        cluster[q] = (d1 < d0) ? 1 : 0;
    }

    // 4) Формируем битовую маску
    int pixels = isl->count * BLOCK_SIZE * BLOCK_SIZE;
    int bytes = (pixels + 7) / 8;
    uint8_t *m = calloc(bytes, 1);

    int bitpos = 0;
    for (int bi = 0; bi < isl->count; ++bi) {
        int b = isl->blocks[bi];
        int by = b / g.block_cols;
        int bx = b % g.block_cols;
        int x0 = bx*BLOCK_SIZE, y0 = by*BLOCK_SIZE;
        for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
            for (int dx = 0; dx < BLOCK_SIZE; ++dx) {
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
    expand_rgb222(c0, &r0, &g0, &b0);

    // ищем в палитре цвет, максимально удалённый от c0
    double maxd = -1.0;
    int best1 = -1;
    for (int i = 0; i < palette_n; ++i) {
        if (i == best0) continue;
        uint8_t ri,gi,bi;
        expand_rgb222(palette[i], &ri, &gi, &bi);
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
        expand_rgb222(q,   &ri,&gi,&bi);
        expand_rgb222(c1, &r1,&g1,&b1);
        double d0 = (r0 - ri)*(r0 - ri) + (g0 - gi)*(g0 - gi) + (b0 - bi)*(b0 - bi);
        double d1 = (r1 - ri)*(r1 - ri) + (g1 - gi)*(g1 - gi) + (b1 - bi)*(b1 - bi);
        cluster[q] = (d1 < d0) ? 1 : 0;
    }

    // 3) сформировать битовую маску: для каждого пикселя острова пишем 0/1
    // -----------------------------------------------------------------
    int pixels = isl->count * BLOCK_SIZE * BLOCK_SIZE;
    int bytes  = (pixels + 7) / 8;
    uint8_t *m = calloc(bytes, 1);
    int bitpos = 0;

    for (int bi = 0; bi < isl->count; ++bi) {
        int b = isl->blocks[bi];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int y0 = by * BLOCK_SIZE, x0 = bx * BLOCK_SIZE;

        for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
            for (int dx = 0; dx < BLOCK_SIZE; ++dx, ++bitpos) {
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
 * @param quant           входной буфер RGB222 [padded_h][padded_w]
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
    int pixels = isl->count * BLOCK_SIZE * BLOCK_SIZE;
    int bytes  = (pixels + 7) / 8;
    uint8_t *mask = calloc(bytes, 1);
    int bitpos = 0;

    for (int bi = 0; bi < isl->count; ++bi) {
        int b  = isl->blocks[bi];
        int by = b / g.block_cols, bx = b % g.block_cols;
        int y0 = by * BLOCK_SIZE, x0 = bx * BLOCK_SIZE;

        for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
            for (int dx = 0; dx < BLOCK_SIZE; ++dx, ++bitpos) {
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
 * @param palette   два байта RGB222-кода: palette[0] и palette[1]
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
        int x0 = bx * BLOCK_SIZE;
        int y0 = by * BLOCK_SIZE;
        minx = x0 < minx ? x0 : minx;
        miny = y0 < miny ? y0 : miny;
        maxx = (x0 + BLOCK_SIZE - 1) > maxx ? x0 + BLOCK_SIZE - 1 : maxx;
        maxy = (y0 + BLOCK_SIZE - 1) > maxy ? y0 + BLOCK_SIZE - 1 : maxy;
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
        int x0 = bx * BLOCK_SIZE;
        int y0 = by * BLOCK_SIZE;

        for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
            int yy = y0 + dy;
            if (yy < miny || yy > maxy) {
                bitpos += BLOCK_SIZE;
                continue;
            }
            for (int dx = 0; dx < BLOCK_SIZE; ++dx, ++bitpos) {
                int xx = x0 + dx;
                if (xx < minx || xx > maxx) continue;

                // какой кластер: 0 или 1
                int cluster = (mask[bitpos >> 3] >> (bitpos & 7)) & 1;
                uint8_t q = palette[cluster];
                // расширяем RGB222 → RGB888
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


static void dump_all_islands_png(
    const char *fname,
    const uint8_t *quant,
    int padded_w,
    const Island *islands,
    int island_n
    ) {
    int W = g.w, H = g.h;
    // 1) создаём фон из quant → RGB888
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { perror("malloc"); return; }
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint8_t q = quant[y * padded_w + x];
            uint8_t r2 = (q >> 4) & 3, g2 = (q >> 2) & 3, b2 = q & 3;
            uint8_t r8 = expand2(r2), g8 = expand2(g2), b8 = expand2(b2);
            size_t p = ((size_t)y * W + x) * 3;
            rgb[p+0] = r8; rgb[p+1] = g8; rgb[p+2] = b8;
        }
    }

    // 2) для каждого острова рисуем рамку
    for (int i = 0; i < island_n; ++i) {
        const Island *isl = &islands[i];
        // вычисляем bbox
        int minx = W, miny = H, maxx = 0, maxy = 0;
        for (int j = 0; j < isl->count; ++j) {
            int b = isl->blocks[j];
            int by = b / g.block_cols, bx = b % g.block_cols;
            int x0 = bx*BLOCK_SIZE, y0 = by*BLOCK_SIZE;
            minx = x0 < minx ? x0 : minx;
            miny = y0 < miny ? y0 : miny;
            maxx = x0+BLOCK_SIZE-1 > maxx ? x0+BLOCK_SIZE-1 : maxx;
            maxy = y0+BLOCK_SIZE-1 > maxy ? y0+BLOCK_SIZE-1 : maxy;
        }
        // clip to screen
        if (minx<0) minx=0;
        if (miny<0) miny=0;
        if (maxx>=W) maxx=W-1;
        if (maxy>=H) maxy=H-1;
        // pick color
        uint8_t rr,gg,bb;
        pick_island_color(i, &rr,&gg,&bb);
        // горизонтали
        for (int x = minx; x <= maxx; ++x) {
            size_t p1 = (size_t)miny * W + x;
            size_t p2 = (size_t)maxy * W + x;
            rgb[p1*3+0]=rr; rgb[p1*3+1]=gg; rgb[p1*3+2]=bb;
            rgb[p2*3+0]=rr; rgb[p2*3+1]=gg; rgb[p2*3+2]=bb;
        }
        // вертикали
        for (int y = miny; y <= maxy; ++y) {
            size_t p1 = (size_t)y * W + minx;
            size_t p2 = (size_t)y * W + maxx;
            rgb[p1*3+0]=rr; rgb[p1*3+1]=gg; rgb[p1*3+2]=bb;
            rgb[p2*3+0]=rr; rgb[p2*3+1]=gg; rgb[p2*3+2]=bb;
        }
    }

    // 3) сохраняем
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
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


            // === Отладочный вывод сырого скриншота перед обработкой
            if (getenv("XCAP_DEBUG_RAW")) {
                // Выделяем буфер RGB888 для сырого кадра
                uint8_t *raw_rgb = malloc((size_t)g.w * g.h * 3);
                if (raw_rgb) {
                    // Конвертация BGRA→RGB
                    for (int y = 0; y < g.h; ++y) {
                        for (int x = 0; x < g.w; ++x) {
                            size_t idx = (size_t)y * g.w + x;
                            uint8_t *px = s->raw + idx * 4;
                            raw_rgb[idx*3 + 0] = px[2]; // R
                            raw_rgb[idx*3 + 1] = px[1]; // G
                            raw_rgb[idx*3 + 2] = px[0]; // B
                        }
                    }
                    char raw_fn[64];
                    struct timespec t2; clock_gettime(CLOCK_REALTIME, &t2);
                    snprintf(raw_fn, sizeof(raw_fn), "dbg_raw_%u_%ld_%09ld.png",
                             i, t2.tv_sec, t2.tv_nsec);
                    dump_png_rgb(raw_fn, g.w, g.h, raw_rgb);
                    // Освобождаем буфер RGB88
                    free(raw_rgb);
                }
            }


            // === Квантование RGB222 + анализ uniform-блоков 8×8
            // Принимает: s->raw - ARGB8888
            // Возвращает:
            //    s->quant,    // RGB222-byte per px
            //    s->color,    // блоки 8×8: uniform-цвет или MIXED (0xFF)
            // Ничего не выделяет, что следовало бы освободить.
            quantize_and_analyze(
                s->raw,      // ARGB8888
                s->quant,    // RGB222-byte per px
                s->color,    // блоки 8×8: uniform-цвет или MIXED (0xFF)
                g.w, g.h);


            // === Отладочный вывод квантованного изображения RGB222
            if (getenv("XCAP_DEBUG_QUANT")) {
                // Выделяем RGB888 буфер
                uint8_t *rgb = malloc((size_t)g.w * g.h * 3);
                if (!rgb) { die("XCAP_DEBUG_QUANT malloc failed"); }
                // Конвертация
                for (int y = 0; y < g.h; ++y) {
                    for (int x = 0; x < g.w; ++x) {
                        uint8_t q = s->quant[y * g.padded_w + x];
                        uint8_t r2 = (q>>4)&0x3;
                        uint8_t g2 = (q>>2)&0x3;
                        uint8_t b2 =  q    &0x3;
                        size_t idx = (size_t)y * g.w + x;
                        rgb[idx*3+0] = EXPAND2(r2);
                        rgb[idx*3+1] = EXPAND2(g2);
                        rgb[idx*3+2] = EXPAND2(b2);
                    }
                }
                // Сохраняем PNG
                char qfn[64];
                struct timespec t0;
                clock_gettime(CLOCK_REALTIME, &t0);
                snprintf(qfn, sizeof(qfn),
                         "dbg_quant_%u_%ld_%09ld.png",
                         i, t0.tv_sec, t0.tv_nsec);
                dump_png_rgb(qfn, g.w, g.h, rgb);
                free(rgb);
            }

            // === Отладочный вывод с отображением залитых и незалитых блоков 8×8
            if (getenv("XCAP_DEBUG_FILL")) {
                char ffn[64];
                struct timespec t1;
                clock_gettime(CLOCK_REALTIME, &t1);
                snprintf(ffn, sizeof(ffn),
                         "dbg_fill_%u_%ld_%09ld.png",
                         i, t1.tv_sec, t1.tv_nsec);
                dump_filled_blocks_png(ffn, s->quant, s->color);
            }

            // === Найти "острова" MIXED-блоков
            // Принимает: s->color - блоки 8×8: uniform-цвет или MIXED (0xFF)
            // Возвращает:
            //    *islands,    // Массив "островов"
            //    island_n,    // Количество найденных "островов"
            // Выделяет:
            //    *islands
            int island_n;
            Island *islands = detect_mixed_islands(
                s->color, g.block_rows, g.block_cols, &island_n);

            // === собрать для "островов" MIXED-блоков палитру

            // 1) Печатаем палитры для каждого острова
            for (int isl_i = 0; isl_i < island_n; ++isl_i) {
                Island *isl = &islands[isl_i];
                uint8_t *palette;   // содержат 8-битные цвета
                int      palette_n; // длина массива palette
                int     *counts;    // cnt[i] — частота цвета palette[i].
                // Собираем палитру и гистограмму частот
                // Принимает:
                //    s->quant - RGB222-byte per px
                //    isl - остров
                // Возвращает:
                //    *palette   - палитру, динамически выделенный массив байт,
                //                 каждый — это уникальный цвет.
                //    *palette_n - количество элементов палитры
                // Выделяет:
                //    *palette
                //    *counts
                collect_palette_hist(
                    s->quant,       // RGB222-byte per px
                    g.padded_w,
                    isl,
                    &palette,
                    &palette_n,
                    &counts
                    );

                // Выводим размер и содержимое палитры
                fprintf(stdout,
                        "[slot %u] island %d: blocks=%d palette=%d:",
                        i, isl_i, (&islands[isl_i])->count, palette_n);
                for (int pi = 0; pi < palette_n; ++pi) {
                    fprintf(stdout, " %02X", palette[pi]);
                }
                fprintf(stdout, "\n");

                /* [TODO:] Здесь нужно сливать острова если один находится внутри другого */

                /* CLASSIFY */

                /* Сортируем индексы цвета palette по убыванию cnt. */
                int order[64]; // массив индексов order, ссылаеющихся на элементы палитры
                if (palette_n > 0 && palette_n <= 64) {
                    /* заполняем массив order */
                    for (int k = 0; k < palette_n; ++k) order[k] = k;
                    // По окончании сортировки
                    // order[0] указывает на наиболее частый цвет,
                    // order [1] — на второй по частоте и так далее
                    _sort_palette = palette;
                    _sort_counts  = counts;
                    qsort(order, palette_n, sizeof *order, cmp_order_desc);
                }

                // Считаем сколько всего пикселей в острове
                int total = 0;
                for (int pi = 0; pi < palette_n; ++pi) {
                    total += counts[ palette[pi] ];
                }
                // Цвет фона - это самый частый цвет
                int bgc = counts[ palette[ order[0] ] ];
                // Цвет текста - скорее всего второй по частоте
                int txc = (palette_n > 1 ? counts[ palette[ order[1] ] ] : 0);
                /* Доля фонового цвета cnt[0]/total_pixels (должна быть большая) */
                float bg_frac = (float)bgc / total;
                /* Доля текста cnt[1]/total_pixels (должна быть ненулевая) */
                float fg_frac = (float)txc / total;

                // Вычисляем минимальный ограничивающий прямоугольник («bounding box»)
                // данного острова в пиксельных координатах
                int minx = g.w, miny = g.h, maxx = 0, maxy = 0;
                for (int j = 0; j < islands[isl_i].count; ++j) {
                    int b = islands[isl_i].blocks[j];
                    int by = b / g.block_cols;
                    int bx = b % g.block_cols;
                    int x0 = bx * BLOCK_SIZE;
                    int y0 = by * BLOCK_SIZE;
                    if (x0 < minx) minx = x0;
                    if (y0 < miny) miny = y0;
                    if (x0 + BLOCK_SIZE - 1 > maxx) maxx = x0 + BLOCK_SIZE - 1;
                    if (y0 + BLOCK_SIZE - 1 > maxy) maxy = y0 + BLOCK_SIZE - 1;
                }
                // Вычисляем отношение сторон
                float shape_ratio = (float)(maxx - minx + 1) / (maxy - miny + 1);
                // Вычисляем похожесть острова на текст
                bool is_text = (palette_n <= 16)
                    && (bg_frac >= 0.6f)
                    && (fg_frac >= 0.03f)
                    /* && (shape_ratio >= 4.0f) */
                    ;

                fprintf(
                    stdout,
                    "[slot %u][island %d] palette=%d total_px=%d bg=%.2f fg=%.2f shape=%.1f => %s\n",
                    i, isl_i, palette_n, total, bg_frac, fg_frac, shape_ratio,
                    is_text ? "TEXT" : "GRAPHIC");

                /* END_CLASSIFY */

                /* // 1e) Отладка: сжать каждый остров 2-цветной палитрой */
                /* uint8_t pal2[2], *mask2; */
                /* int mask_bytes2; */
                /* compress_island_2col_simple( */
                /*     s->quant, g.padded_w, */
                /*     &islands[isl_i], */
                /*     palette, palette_n, */
                /*     counts, */
                /*     pal2, &mask2, &mask_bytes2 */
                /*     ); */
                /* fprintf( */
                /*     stdout, */
                /*     "[slot %u][island %d] compressed → palette=(%02X,%02X) mask_bytes=%d\n", */
                /*     i, isl_i, pal2[0], pal2[1], mask_bytes2 */
                /*     ); */

                /* // 1f) Отладочный дамп: распаковать и сохранить остров */
                /* if (getenv("XCAP_DEBUG_DECOMPRESS")) { */
                /*     char dfn[64]; */
                /*     struct timespec dt; */
                /*     clock_gettime(CLOCK_REALTIME, &dt); */
                /*     snprintf(dfn, sizeof(dfn), */
                /*              "dbg_decomp_%u_%d_%ld.png", */
                /*              i, isl_i, dt.tv_sec); */
                /*     decompress_island_to_png( */
                /*         dfn, */
                /*         &islands[isl_i], */
                /*         g.padded_w, */
                /*         pal2, */
                /*         mask2 */
                /*         ); */
                /* } */

                // Здесь мы заканчиваем работу с палитрой и ее можно освободить
                free(palette);
                free(counts);
                /* free(mask2); */

                /* */
            }


            // 2) Дополнительно можно вывести PNG с островами
            char fn[64];
            struct timespec t;
            clock_gettime(CLOCK_REALTIME, &t);
            snprintf(fn, sizeof(fn), "dbg_islands_%u_%ld.png", i, t.tv_sec);
            printf("-- out no 1\n");

            dump_islands_png(fn, s->quant, islands, island_n);

            printf("-- out no 2\n");


            // 1g) Отладка: вывести все острова на общей картинке
            if (getenv("XCAP_DEBUG_ALL_ISLANDS")) {
                char afn[64];
                struct timespec ta;
                clock_gettime(CLOCK_REALTIME, &ta);
                snprintf(afn, sizeof(afn),
                         "dbg_all_islands_%u_%ld.png",
                         i, ta.tv_sec);
                dump_all_islands_png(
                    afn,
                    s->quant,
                    g.padded_w,
                    islands,
                    island_n
                    );
            }



            // 3) Теперь очищаем все blocks и сам массив islands
            for (int isl_i = 0; isl_i < island_n; ++isl_i) {
                free(islands[isl_i].blocks);
            }

            printf("-- out no 3\n");

            free(islands);

            printf("-- out no 4\n");



            /* // 2) Отладочный вывод: записать информацию по каждому блоку в файл */
            /* // Имя файла: dbg_blocks_<slot>_<timestamp>.txt */
            /* struct timespec ts; */
            /* clock_gettime(CLOCK_REALTIME, &ts); */
            /* snprintf(dbg_fname, sizeof(dbg_fname), */
            /*          "dbg_blocks_%u_%ld_%09ld.txt", */
            /*          i, ts.tv_sec, ts.tv_nsec); */

            /* FILE *df = fopen(dbg_fname, "w"); */
            /* if (df) { */
            /*     fprintf(df, "DEBUG BLOCKS SLOT %u (worker %lu)\n", i, (unsigned long)wid); */
            /*     fprintf(df, "Screen: %dx%d, padded: %dx%d\n", */
            /*             g.w, g.h, g.padded_w, g.padded_h); */
            /*     fprintf(df, "Blocks: %d cols × %d rows = %d total\n\n", */
            /*             g.block_cols, g.block_rows, g.block_count); */

            /*     for (int b = 0; b < g.block_count; ++b) { */
            /*         uint8_t c = s->color[b]; */
            /*         int col = b % g.block_cols; */
            /*         int row = b / g.block_cols; */
            /*         fprintf(df, "Block %3d at [%2d,%2d]: color=0x%02X %s\n", */
            /*                 b, row, col, c, */
            /*                 (c == 0xFF ? "(MIXED)" : "(UNIFORM)")); */

            /*         if (c == 0xFF) { */
            /*             // Если блок смешанный, вывести все 8×8 квантованных пикселей */
            /*             fprintf(df, "  Pixels:\n    "); */
            /*             int base_y = row * 8; */
            /*             int base_x = col * 8; */
            /*             for (int y = 0; y < 8; ++y) { */
            /*                 for (int x = 0; x < 8; ++x) { */
            /*                     int idx = (base_y + y) * g.padded_w + (base_x + x); */
            /*                     uint8_t q = s->quant[idx]; */
            /*                     fprintf(df, "%02X ", q); */
            /*                 } */
            /*                 if (y < 7) fprintf(df, "\n    "); */
            /*             } */
            /*             fprintf(df, "\n"); */
            /*         } */
            /*     } */
            /*     fclose(df); */
            /* } else { */
            /*     fprintf(stderr, "[worker %lu] failed to open debug file %s: %s\n", */
            /*             (unsigned long)wid, dbg_fname, strerror(errno)); */
            /* } */


            /* // 1a) Отладочный вывод результата в PNG, если установлен XCAP_DEBUG */
            /* if (getenv("XCAP_DEBUG")) { */
            /*     // Буфер RGB888 для дампа: ширина*высота*3 */
            /*     uint8_t *dbg_rgb = malloc((size_t)g.w * g.h * 3); */
            /*     if (dbg_rgb) { */
            /*         // (Функция EXPAND2(v)расширяет 2-бит -> 8-бит) */
            /*         /\* // Заполнение пикселей *\/ */
            /*         /\* for (int y = 0; y < g.h; ++y) { *\/ */
            /*         /\*     for (int x = 0; x < g.w; ++x) { *\/ */
            /*         /\*         int blk = (y/8) * g.block_cols + (x/8); *\/ */
            /*         /\*         uint8_t bc = s->color[blk]; *\/ */
            /*         /\*         uint8_t r, gcol, b; *\/ */
            /*         /\*         if (bc == MIXED) { *\/ */
            /*         /\*             // Смешанный: показываем оригинальный квант *\/ */
            /*         /\*             uint8_t q = s->quant[y * g.padded_w + x]; *\/ */
            /*         /\*             uint8_t r2 = (q>>4)&0x3; *\/ */
            /*         /\*             uint8_t g2 = (q>>2)&0x3; *\/ */
            /*         /\*             uint8_t b2 =  q    &0x3; *\/ */
            /*         /\*             r    = EXPAND2(r2); *\/ */
            /*         /\*             gcol = EXPAND2(g2); *\/ */
            /*         /\*             b    = EXPAND2(b2); *\/ */
            /*         /\*         } else if (bc & TRANSP_BIT) { *\/ */
            /*         /\*             // Прозрачные: чёрный *\/ */
            /*         /\*             r = gcol = b = 0; *\/ */
            /*         /\*         } else { *\/ */
            /*         /\*             // Однородный: весь блок в цвет bc *\/ */
            /*         /\*             uint8_t r2 = (bc>>4)&0x3; *\/ */
            /*         /\*             uint8_t g2 = (bc>>2)&0x3; *\/ */
            /*         /\*             uint8_t b2 =  bc    &0x3; *\/ */
            /*         /\*             r    = EXPAND2(r2); *\/ */
            /*         /\*             gcol = EXPAND2(g2); *\/ */
            /*         /\*             b    = EXPAND2(b2); *\/ */
            /*         /\*         } *\/ */
            /*         /\*         size_t off = ((size_t)y * g.w + x) * 3; *\/ */
            /*         /\*         dbg_rgb[off + 0] = r; *\/ */
            /*         /\*         dbg_rgb[off + 1] = gcol; *\/ */
            /*         /\*         dbg_rgb[off + 2] = b; *\/ */
            /*         /\*     } *\/ */
            /*         /\* } *\/ */

            /*         // 1b) Сначала заполняем фон визуализацией квантованного изображения */
            /*         for (int y = 0; y < g.h; ++y) { */
            /*             for (int x = 0; x < g.w; ++x) { */
            /*                 uint8_t q = s->quant[y * g.padded_w + x]; */
            /*                 uint8_t r2 = (q>>4)&0x3; */
            /*                 uint8_t g2 = (q>>2)&0x3; */
            /*                 uint8_t b2 =  q    &0x3; */
            /*                 uint8_t r = EXPAND2(r2); */
            /*                 uint8_t gcol = EXPAND2(g2); */
            /*                 uint8_t b = EXPAND2(b2); */
            /*                 size_t off = ((size_t)y * g.w + x) * 3; */
            /*                 dbg_rgb[off + 0] = r; */
            /*                 dbg_rgb[off + 1] = gcol; */
            /*                 dbg_rgb[off + 2] = b; */
            /*             } */
            /*         } */
            /*         // Затем накладываем сетку блоков */
            /*         for (int x = 0; x < g.w; x += 8) { */
            /*             for (int y = 0; y < g.h; ++y) { */
            /*                 size_t off = ((size_t)y * g.w + x) * 3; */
            /*                 // красная вертикаль */
            /*                 dbg_rgb[off + 0] = 255; dbg_rgb[off + 1] = 0; dbg_rgb[off + 2] = 0; */
            /*             } */
            /*         } */
            /*         for (int y = 0; y < g.h; y += 8) { */
            /*             for (int x = 0; x < g.w; ++x) { */
            /*                 size_t off = ((size_t)y * g.w + x) * 3; */
            /*                 // зелёная горизонталь */
            /*                 dbg_rgb[off + 0] = 0; dbg_rgb[off + 1] = 255; dbg_rgb[off + 2] = 0; */
            /*             } */
            /*         } */


/* // Сохранить PNG */
/*                     char png_fn[64]; */
/*                     struct timespec ts; */
/*                     clock_gettime(CLOCK_REALTIME, &ts); */
/*                     snprintf(png_fn, sizeof(png_fn), */
/*                              "dbg_blocks_%u_%ld_%09ld.png", */
/*                              i, ts.tv_sec, ts.tv_nsec); */
/*                     dump_png_rgb(png_fn, g.w, g.h, dbg_rgb); */
/*                     free(dbg_rgb); */
/*                 } */
/*             } */

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
    g.block_cols  = (g.w + BLOCK_SIZE - 1) / BLOCK_SIZE;
    g.block_rows  = (g.h + BLOCK_SIZE - 1) / BLOCK_SIZE;
    g.block_count = g.block_cols * g.block_rows;
    g.padded_w    = g.block_cols * BLOCK_SIZE;  // теперь ≥ w и кратно 32
    g.padded_h    = g.block_rows * BLOCK_SIZE;  // теперь ≥ h и кратно 32

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
