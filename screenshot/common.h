#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <png.h>
#include <math.h>
#include <limits.h>
#include <emmintrin.h>  // SSE2
#include <immintrin.h>  // AVX2
#include <xmmintrin.h>  // SSE

/* Для определения количества ядер в get_num_cores */
#if defined(_WIN32)
    #include <windows.h>
#elif defined(__linux__)
    #include <unistd.h>
#elif defined(__APPLE__)
    #include <sys/types.h>
    #include <sys/sysctl.h>
#else
    #error "Unsupported platform"
#endif


/* ========== Определение платформы ========== */

#if defined(_WIN32)
    #ifndef PLATFORM_WINDOWS
        #define PLATFORM_WINDOWS
    #endif
    #include <windows.h>    // QueryPerformanceCounter и пр.

#elif defined(__linux__)
    #ifndef PLATFORM_LINUX
        #define PLATFORM_LINUX
    #endif
    #include <numa.h>       // libnuma

#elif defined(__APPLE__)
    #ifndef PLATFORM_MACOS
        #define PLATFORM_MACOS
    #endif

#else
    #error "Unsupported platform"
#endif

/* ========== Константы ========== */

#define BS 32
#define MAX_SLOTS 8


/* ========== Пути сохранения скриншотов ========== */


#ifndef SCREENSHOT_PATH
    #ifdef PLATFORM_SHOT_PATH
        #define SCREENSHOT_PATH PLATFORM_SHOT_PATH

    #elif defined(PLATFORM_WINDOWS) || defined(_WIN32)
        #define SCREENSHOT_PATH "C:\\Tmp\\"

    #elif defined(PLATFORM_LINUX)   || defined(__linux__)
        #define SCREENSHOT_PATH "/tmp/"

    #elif defined(PLATFORM_MACOS)   || defined(__APPLE__)
        #define SCREENSHOT_PATH "/Users/Shared/tmp/"

    #else
        #define SCREENSHOT_PATH "./"
    #endif  /* PLATFORM_SHOT_PATH / PLATFORM_* */

#endif  /* SCREENSHOT_PATH */


// Максимальная длина полного пути к файлу
#define MAX_SCREENSHOT_PATH 512
#define DEFAULT_SLOTS 4
#define MIXED 0xFF
#define MAX_NEIGHBOR_GAP 32
#define MAX_HEIGHT_DIFF 6
#define MAX_VERTICAL_OFFSET (BS/2)
#define TEMPLATE_SIZE 16
#define TEMPLATE_PIXELS (TEMPLATE_SIZE * TEMPLATE_SIZE)
#define TEMPLATE_BYTES ((TEMPLATE_PIXELS + 7) / 8)
#define MATCH_THRESHOLD 0.75f
#define MAX_REGIONS 1024
#define COLOR_MIXED 0xFFFF

#define STALL_NS 1000000000ull
#define MAGIC_QIMG 0x51494D47u
#define ALIGN 128

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif


/* ========== Общие типы ========== */

enum slot_state { FREE, RAW_READY, IN_PROGRESS, QUANT_DONE, SERIALIZING };


typedef struct {
    _Atomic enum slot_state st;
    uint64_t  t_start;
    uint8_t  *raw;         // указатель на сырое изображение RGB/BGR от платформы
    uint8_t  *quant;       // RGB332 (один байт на пиксель)
    uint8_t  *bg;          // фоновой цвет каждого блока
    uint8_t  *fg;          // цвет текста - второй по распространенности
    uint8_t  *mask;        // битовая маска блоков
    int       block_count; // число блоков
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

    /* платформо-зависимые данные */
    void *platform_data;
} GlobalContext;


typedef struct {
    uint16_t by, bx;  // block indices
    uint8_t  dy, dx;  // pixel offsets inside block
} Pixel;


typedef struct OcrRegion {
    int minx, miny, maxx, maxy;  // bounding box in global pixels
    int count;                   // number of pixels in region
    Pixel *pixels;               // array of Pixel
    struct OcrRegion *neighbor;  // next region on the right, or NULL
} OcrRegion;


typedef struct {
    OcrRegion **regions;  // указатели на регионы в этой строке
    int      count;    // сколько регионов в строке
    int minx, miny, maxx, maxy; // computed bbox
} Line;


typedef struct {
    char ch;
    uint8_t mask[TEMPLATE_BYTES];
} CharTemplate;


/* ========== Глобальные переменные ========== */

extern CharTemplate templates[];
extern int n_templates;


/* ========== Общие функции ========== */

// Утилиты времени
static inline uint64_t now_ns(void) {
#ifdef PLATFORM_WINDOWS
    LARGE_INTEGER frequency, counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return (uint64_t)((counter.QuadPart * 1000000000ull) / frequency.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
#endif
}

static inline void die(const char *m) {
    perror(m);
    exit(EXIT_FAILURE);
}

// Цветовые утилиты
static inline uint8_t expand2(uint8_t v) {
    return (uint8_t)((v << 6) | (v << 4) | (v << 2) | v);
}

static inline uint8_t expand3(uint8_t v) {
    return (uint8_t)((v << 5) | (v << 2) | (v >> 1));
}

// Выравнивание памяти
static inline size_t align_up(size_t x) {
    return (x + (ALIGN - 1)) & ~(ALIGN - 1);
}

// SIMD проверки
static inline bool cpu_has_avx2(void) {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    uint32_t a, b, c, d;
    __asm__ volatile("cpuid"
                     : "=a"(a), "=b"(b), "=c"(c), "=d"(d)
                     : "a"(7), "c"(0)
        );
    return (b & (1<<5)) != 0;  // EBX[5] = AVX2
#else
    return false;
#endif
}

// Объявления платформо-зависимых функций
bool platform_init(GlobalContext *ctx);
void platform_cleanup(GlobalContext *ctx);
bool platform_capture_screen(GlobalContext *ctx, int slot_index);

// Platform-specific квантизация изображений
void platform_quantize_bgr_to_rgb332(const uint8_t *bgr_data, uint8_t *quant_data,
                                      int width, int height, int padded_width);
void platform_quantize_rgba_to_rgb332(const uint8_t *rgba_data, uint8_t *quant_data,
                                       int width, int height, int padded_width, int bytes_per_line);

// Объявления общих функций обработки
void allocate_bigmem(GlobalContext *ctx);
void analyze_blocks(FrameSlot *slot, GlobalContext *ctx);
void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb);

// Функции работы с путями
void create_screenshot_path(char *full_path, size_t path_size, const char *filename);

// SIMD функции
bool block_uniform_avx2(const uint8_t *quant, int pw, int sy, int sx);

// Обработка изображений
OcrRegion* detect_regions(const uint8_t *mask, const uint8_t *block_color,
                       int block_rows, int block_cols, int *out_region_n);
void group_regions(OcrRegion *regions, int region_n, Line **out_lines, int *out_line_n);
void set_region_neighbors(OcrRegion *regions, int region_n);

// Распознавание текста
void render_region(const OcrRegion *reg, uint8_t *sample);
char recognize_region(const OcrRegion *reg);

// Отладочные функции
void debug_dump_quant(int slot, const uint8_t *quant, int padded_w, GlobalContext *ctx);
void debug_dump_filled(int slot_idx, const FrameSlot *slot, GlobalContext *ctx);
void debug_dump_regions(int slot_idx, const OcrRegion *regions, int region_n, GlobalContext *ctx);
void debug_recognize(int slot_idx, OcrRegion *regions, int region_n);
void debug_dump_lines(int slot_idx, Line *lines, int line_n, GlobalContext *ctx);
void debug_dump_chains(int slot_idx, OcrRegion *regions, int region_n, GlobalContext *ctx);
void dump_rgba_as_bmp(const char *fname, int W, int H, const uint8_t *rgba);

// Утилиты
static inline int popcount8(uint8_t v) {
#ifdef __GNUC__
    return __builtin_popcount(v);
#else
    // Fallback implementation
    int count = 0;
    while (v) {
        count += v & 1;
        v >>= 1;
    }
    return count;
#endif
}



/**
 * Определяет число логических процессорных ядер
 */
static inline int get_num_cores(void) {
#if defined(_WIN32)
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    return (int)info.dwNumberOfProcessors;

#elif defined(__linux__)
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return (n > 0 ? (int)n : 1);

#elif defined(__APPLE__)
    int mib[2];
    uint32_t count;
    size_t len = sizeof(count);

    mib[0] = CTL_HW;
    mib[1] = HW_NCPU;
    if (sysctl(mib, 2, &count, &len, NULL, 0) == 0 && count > 0) {
        return (int)count;
    } else {
        return 1;
    }
#endif
}


#endif /* COMMON_H */
