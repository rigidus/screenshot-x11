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
/* ========== Определение платформы ========== */
#ifdef _WIN32
    #define PLATFORM_WINDOWS
    #include <windows.h>  // для QueryPerformanceCounter
#elif defined(__linux__)
    #define PLATFORM_LINUX
    #include <numa.h>
#else
    #error "Unsupported platform"
#endif

/* ========== Константы ========== */
#define BS 32
#define MAX_SLOTS 8
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
#ifdef PLATFORM_WINDOWS
    #define PIPE_PATH_BASE "screenshot_pipe"
#else
    #define PIPE_PATH "/tmp/screenshot_pipe"
#endif
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
    uint8_t  *rgba;        // унифицированные RGBA данные (4 байта на пиксель)
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

typedef struct Region {
    int minx, miny, maxx, maxy;  // bounding box in global pixels
    int count;                   // number of pixels in region
    Pixel *pixels;               // array of Pixel
    struct Region *neighbor;     // next region on the right, or NULL
} Region;

typedef struct {
    Region **regions;  // указатели на регионы в этой строке
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

// Конвертация форматов
void convert_bgr_to_rgba(const uint8_t *bgr_data, uint8_t *rgba_data, int width, int height);

// Объявления общих функций обработки
void allocate_bigmem(GlobalContext *ctx);
void quantize_and_analyze(FrameSlot *slot, GlobalContext *ctx);
void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb);

// SIMD функции
bool block_uniform_avx2(const uint8_t *quant, int pw, int sy, int sx);

// Обработка изображений
Region* detect_regions(const uint8_t *mask, const uint8_t *block_color,
                       int block_rows, int block_cols, int *out_region_n);
void group_regions(Region *regions, int region_n, Line **out_lines, int *out_line_n);
void set_region_neighbors(Region *regions, int region_n);

// Распознавание текста
void render_region(const Region *reg, uint8_t *sample);
char recognize_region(const Region *reg);

// Отладочные функции
void debug_dump_quant(int slot, const uint8_t *quant, int padded_w, GlobalContext *ctx);
void debug_dump_filled(int slot_idx, const FrameSlot *slot, GlobalContext *ctx);
void debug_dump_regions(int slot_idx, const Region *regions, int region_n, GlobalContext *ctx);
void debug_recognize(int slot_idx, Region *regions, int region_n);
void debug_dump_lines(int slot_idx, Line *lines, int line_n, GlobalContext *ctx);
void debug_dump_chains(int slot_idx, Region *regions, int region_n, GlobalContext *ctx);
void dump_rgba_as_bmp(const char *fname, int W, int H, const uint8_t *rgba);
void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb);

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

#endif /* COMMON_H */
