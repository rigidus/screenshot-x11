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


/* ---------- константы и вспомогалки ------------------------------------ */
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


/* ═════════════════════════ SIMD RGB888→RGB222 ═════════════════════════════ */

#define BLOCK_SIZE    8
#define MIXED         0xFF
#define TRANSP_BIT    0x80


// Проверка поддержки AVX2 (разовая, в init)
static bool have_avx2(void) {
    uint32_t a, b, c, d;
    __asm__ volatile(
        "cpuid\n\t"
        : "=a"(a), "=b"(b), "=c"(c), "=d"(d)
        : "a"(0), "c"(0)
        );
    // Узнаём feature bits через CPUID leaf 7 subleaf 0
    __asm__ volatile(
        "cpuid\n\t"
        : "=a"(a), "=b"(b), "=c"(c), "=d"(d)
        : "a"(7), "c"(0)
        );
    return (b & (1 << 5)) != 0; // AVX2 bit in EBX
}

// Расширение 2-бит → 8-бит
static inline uint8_t expand2(uint8_t v) {
    return (uint8_t)((v << 6) | (v << 4) | (v << 2) | v);
}

static void quantize_and_analyze(
    const uint8_t *src0,     // ARGB8888, stride = w*4
    uint8_t       *quant,    // буфер [padded_w * padded_h]
    uint8_t       *block_color, // буфер [block_count]
    int             w,
    int             h
    ) {
    int pw = g.padded_w;
    int ph = g.padded_h;
    int bc = g.block_cols;
    int br = g.block_rows;

    // Инициализация цветов блоков
    for (int i = 0; i < br * bc; ++i) {
        block_color[i] = MIXED;
    }

    // Обход по строкам: отдельно для src (ширина w) и quant (ширина pw)
    for (int y = 0; y < ph; ++y) {
        const uint8_t *row_src = src0 + (size_t)y * w * 4;
        uint8_t       *row_q   = quant + (size_t)y * pw;
        int x = 0;

        // --- векторное квантование ---
        if (have_avx2()) {
            __m256i mask2 = _mm256_set1_epi32(0xC0);
            // AVX2: по 8 пикселей
            for (; x + 8 <= w; x += 8) {
                __m256i pix = _mm256_loadu_si256((__m256i*)(row_src + x*4));
                __m256i r = _mm256_and_si256(_mm256_srli_epi32(pix,16), mask2);
                __m256i g2= _mm256_and_si256(_mm256_srli_epi32(pix, 8), mask2);
                __m256i b = _mm256_and_si256(pix,               mask2);
                __m256i rg = _mm256_or_si256(r, _mm256_srli_epi32(g2,2));
                __m256i rgb= _mm256_or_si256(rg, _mm256_srli_epi32(b,4));
                __m128i lo = _mm256_castsi256_si128(rgb);
                __m128i hi = _mm256_extracti128_si256(rgb,1);
                __m128i p16 = _mm_packus_epi32(lo, hi);
                __m128i p8  = _mm_packus_epi16(p16, _mm_setzero_si128());
                _mm_storel_epi64((__m128i*)(row_q + x), p8);
            }
        } else {
            __m128i mask2s = _mm_set1_epi32(0xC0);
            // SSE2: по 4 пикселя
            for (; x + 4 <= w; x += 4) {
                __m128i pix = _mm_loadu_si128((__m128i*)(row_src + x*4));
                __m128i r = _mm_and_si128(_mm_srli_epi32(pix,16), mask2s);
                __m128i g2= _mm_and_si128(_mm_srli_epi32(pix, 8), mask2s);
                __m128i b = _mm_and_si128(pix,               mask2s);
                __m128i rg   = _mm_or_si128(r,    _mm_srli_epi32(g2,2));
                __m128i rgb  = _mm_or_si128(rg,   _mm_srli_epi32(b, 4));
                __m128i p16  = _mm_packus_epi32(rgb, _mm_setzero_si128());
                __m128i p8   = _mm_packus_epi16(p16, _mm_setzero_si128());
                uint32_t v   = _mm_cvtsi128_si32(p8);
                *(uint32_t*)(row_q + x) = v;
            }
        }
        // остаток по пикселям
        for (; x < w; ++x) {
            uint8_t R = row_src[x*4 + 2];
            uint8_t G = row_src[x*4 + 1];
            uint8_t B = row_src[x*4 + 0];
            row_q[x] = (uint8_t)(((R>>6)<<4) | ((G>>6)<<2) | (B>>6));
        }
        // padding вправо
        for (; x < pw; ++x) {
            row_q[x] = TRANSP_BIT;
        }
    }

    // гарантируем завершение non-temporal store
    _mm_sfence();

    // --- анализ однородности блоков 8×8 ---
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {
            int idx = by * bc + bx;
            int sy = by * BLOCK_SIZE;
            int sx = bx * BLOCK_SIZE;
            uint8_t base = MIXED;

            // найти первый непустой пиксель
            for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
                for (int dx = 0; dx < BLOCK_SIZE; ++dx) {
                    uint8_t q = quant[(sy+dy)*pw + (sx+dx)];
                    if (q & TRANSP_BIT) continue;
                    base = q;
                    goto found;
                }
            }
        found:
            if (base == MIXED) {
                block_color[idx] = TRANSP_BIT;  // полностью прозрачный/mixed
                continue;
            }
            // проверить, не отличаются ли остальные
            for (int dy = 0; dy < BLOCK_SIZE; ++dy) {
                for (int dx = 0; dx < BLOCK_SIZE; ++dx) {
                    uint8_t q = quant[(sy+dy)*pw + (sx+dx)];
                    if ((q & TRANSP_BIT) || q == base) continue;
                    base = MIXED;
                    goto done;
                }
            }
        done:
            block_color[idx] = base;
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


void dump_quantized_rgb222_png(const char *fname,
                               const uint8_t *quant,
                               int W, int H,
                               int padded_w, int padded_h) {
    padded_h = (int)padded_h;
    // Выделяем RGB888 буфер
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) { perror("malloc"); return; }
    // Конвертация
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint8_t q = quant[y * padded_w + x];
            uint8_t r2 = (q>>4)&0x3;
            uint8_t g2 = (q>>2)&0x3;
            uint8_t b2 =  q    &0x3;
            size_t idx = (size_t)y * W + x;
            rgb[idx*3+0] = EXPAND2(r2);
            rgb[idx*3+1] = EXPAND2(g2);
            rgb[idx*3+2] = EXPAND2(b2);
        }
    }
    // Сохраняем PNG
    dump_png_rgb(fname, W, H, rgb);
    free(rgb);
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

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

static void dump_filled_blocks_png(const char *fname,
                                   const uint8_t *quant,
                                   const uint8_t *block_color)
{
    int W = g.w, H = g.h;
    int pw = g.padded_w;
    int bc = g.block_cols, br = g.block_rows;

    // Буфер RGB888
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
                        uint8_t r8 = expand2(r2);
                        uint8_t g8 = expand2(g2);
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
                uint8_t r8 = expand2(r2);
                uint8_t g8 = expand2(g2);
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

            // Отладочный вывод сырого скриншота перед обработкой
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
                    free(raw_rgb);
                }
            }

            // 1) Квантование RGB222 + анализ uniform-блоков 8×8
            quantize_and_analyze(
                s->raw,      // ARGB8888
                s->quant,    // RGB222-byte per px
                s->color,     // блоки 8×8: uniform-цвет или MIXED (0xFF)
                g.w,
                g.h);


            // 1a) Отладочный вывод чистого квантованного изображения RGB222
            if (getenv("XCAP_DEBUG_QUANT")) {
                char qfn[64];
                struct timespec t0;
                clock_gettime(CLOCK_REALTIME, &t0);
                snprintf(qfn, sizeof(qfn),
                         "dbg_quant_%u_%ld_%09ld.png",
                         i, t0.tv_sec, t0.tv_nsec);
                dump_quantized_rgb222_png(
                    qfn,
                    s->quant,
                    g.w, g.h,
                    g.padded_w, g.padded_h
                    );
            }

            /* 1c) Отладочный дамп: залитые блоки 8×8 */
            if (getenv("XCAP_DEBUG_FILL")) {
                char ffn[64];
                struct timespec t1;
                clock_gettime(CLOCK_REALTIME, &t1);
                snprintf(ffn, sizeof(ffn),
                         "dbg_fill_%u_%ld_%09ld.png",
                         i, t1.tv_sec, t1.tv_nsec);
                dump_filled_blocks_png(ffn, s->quant, s->color);
            }


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

    // 4. Вычисление размеров блоков 8×8 и паддинга
    g.block_cols  = (g.w + 7) / 8;
    g.block_rows  = (g.h + 7) / 8;
    g.block_count = g.block_cols * g.block_rows;
    g.padded_w    = g.block_cols * 8;
    g.padded_h    = g.block_rows * 8;

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
