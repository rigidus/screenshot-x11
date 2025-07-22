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


/* #if defined(__SSE2__) || defined(__AVX2__) */
/* …интринсики… */
/* #endif */

#if defined(__x86_64__) || defined(_M_X64)  /* только под x86 */
    #include <emmintrin.h>  // SSE2
    #include <immintrin.h>  // AVX2
    #include <xmmintrin.h>  // SSE
#endif


/* ========== Определение платформы ========== */

#ifdef PLATFORM_SHOT_PATH
#   define SCREENSHOT_PATH PLATFORM_SHOT_PATH
#endif

#if defined(_WIN32)
#   ifndef PLATFORM_WINDOWS
#       define PLATFORM_WINDOWS
#   endif
#   include <windows.h>
    int sched_yield(void);
#   ifndef SCREENSHOT_PATH
#       define SCREENSHOT_PATH "C:\\Tmp\\"
#   endif
#elif defined(__linux__)
#   ifndef PLATFORM_LINUX
#       define PLATFORM_LINUX
#   endif
#   include <numa.h>
#   include <pthread.h>
#   include <sched.h>
#   include <sys/sysinfo.h>
#   include <unistd.h>
#   ifndef SCREENSHOT_PATH
#       define SCREENSHOT_PATH "/tmp/"
#   endif
#elif defined(__APPLE__)
#   ifndef PLATFORM_MACOS
#       define PLATFORM_MACOS
#   endif
#   include <pthread.h>
#   include <mach/mach.h>
#   include <mach/host_info.h>
#   include <unistd.h>
#   include <sys/types.h>
#   include <sys/sysctl.h>
    int sched_yield(void);
#   ifndef SCREENSHOT_PATH
#       define SCREENSHOT_PATH "/Users/Shared/tmp/"
#   endif
#else
#   error "Unsupported platform"
#endif


/* ========== Константы ========== */

#define BS 32
#define MAX_SLOTS 8
#define MAX_WORKERS 32

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



#define MAX_SCREENSHOT_PATH 512 // Максимальная длина полного пути к файлу
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
#define ALIGN_UP 128

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
    return (x + (ALIGN_UP - 1)) & ~(ALIGN_UP - 1);
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
bool platform_init(GlobalContext *ctx, int slots_arg);
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


/* Утилиты */


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


/* Получение доступной физической памяти */
static inline size_t get_free_memory_bytes(void) {
#if defined(PLATFORM_LINUX)
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        return (size_t)info.freeram * info.mem_unit;
    }
    return 0;
#elif defined(PLATFORM_WINDOWS)
    MEMORYSTATUSEX st;
    st.dwLength = sizeof(st);
    if (GlobalMemoryStatusEx(&st)) {
        return (size_t)st.ullAvailPhys;
    }
    return 0;
#elif defined(PLATFORM_MACOS)
    mach_msg_type_number_t count = HOST_VM_INFO_COUNT;
    vm_statistics_data_t vmstat;
    if (host_statistics(mach_host_self(), HOST_VM_INFO, (host_info_t)&vmstat, &count) == KERN_SUCCESS) {
        vm_size_t page_size;
        host_page_size(mach_host_self(), &page_size);
        return (size_t)(vmstat.free_count + vmstat.inactive_count) * page_size;
    }
    return 0;
#else
    return 0;
#endif
}


/* Получение числа доступных CPU ядер */
static inline int get_cpu_count(void) {
#if defined(PLATFORM_LINUX) || defined(PLATFORM_MACOS)
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return (n > 0) ? (int)n : 1;
#elif defined(PLATFORM_WINDOWS)
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    return (int)info.dwNumberOfProcessors;
#else
    return 1;
#endif
}


/**
 * align_up_to_page
 * ----------------
 * Выравнивает x вверх до границы страницы памяти.
 */
static inline size_t align_up_to_page(size_t x) {
#if defined(PLATFORM_LINUX) || defined(PLATFORM_MACOS)
    size_t page = (size_t)sysconf(_SC_PAGESIZE);
    return (x + page - 1) & ~(page - 1);
#elif defined(PLATFORM_WINDOWS)
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    size_t page = (size_t)info.dwPageSize;
    return (x + page - 1) & ~(page - 1);
#else
    return x;
#endif
}

/**
 * calculate_slots
 * ------------------
 * Вычисляет оптимальное число слотов так, чтобы
 * общий объём bigmem + raw-буферов не превышал 1/3 свободной памяти.
 * Для каждой ОС учитывается:
 *  - "raw" буфер (XShm или DIB) в аллокации platform_init
 *  - bigmem (allocate_bigmem): quant, bg, fg, mask + выравнивания
 * Выводит промежуточные результаты в консоль.
 */
static inline int calculate_slots(GlobalContext *ctx) {
    // 1) Получаем свободную память
    size_t free_mem = get_free_memory_bytes();
    const size_t reserve = 100UL * 1024 * 1024; // резерв 100MiB
    size_t usable_mem = (free_mem > reserve ? free_mem - reserve : 0);
    usable_mem /= 3; // используем не более третьей части
    printf("[calc_slots] free_mem=%zu, reserve=%zu, usable_mem(third)=%zu\n",
           free_mem, reserve, usable_mem);

    // 2) "Raw" буфер на слот
    size_t raw_sz = 0;
#if defined(PLATFORM_LINUX)
    // в platform_init mmap без выравнивания, но XShm использует page_size
    raw_sz = align_up_to_page((size_t)ctx->w * ctx->h * 4);
    printf("[calc_slots] raw_sz(per slot) = %zu bytes (%d*%d*4)\n",
           raw_sz, ctx->w, ctx->h);
#elif defined(PLATFORM_WINDOWS)
    raw_sz = align_up_to_page((size_t)ctx->w * ctx->h * 3);
    printf("[calc_slots] raw_sz(per slot) = %zu bytes (%d*%d*3)\n",
           raw_sz, ctx->w, ctx->h);
#elif defined(PLATFORM_MACOS)
    // macOS CGImage не учитывается в bigmem
    raw_sz = 0;
#endif

    // 3) bigmem per slot (как в allocate_bigmem):

    // размер квант-буфера (1 байт на пиксель (RGB332)) с паддингом
    size_t q_sz = (size_t)ctx->padded_w * ctx->padded_h * 1;
    // размер буфера цветов фона всех блоков (1 байт на цвет)
    size_t bg_sz = ctx->block_count;
    // размер буфера цветов текста всех блоков (1 байт на цвет)
    size_t fg_sz = ctx->block_count;
    // размер буфера битовой маски одного блока
    size_t single_mask = (BS * BS) / 8;
    // размер буфера битовых масок всех блоков
    size_t mask_sz = ctx->block_count * single_mask;

    // смещения с учётом выравниваний
    size_t offset_quant = 0;
    size_t offset_bg    = align_up(offset_quant + q_sz);
    size_t offset_fg    = align_up(offset_bg + bg_sz);
    size_t offset_mask  = align_up(offset_fg + fg_sz);

    // размер памяти одного слота (плюс raw)
    size_t per_slot = align_up(offset_mask + mask_sz) + raw_sz;

    printf("[calc_slots] q=%zu bg=%zu fg=%zu mask=%zu | per_slot=%zu bytes\n",
           q_sz, bg_sz, fg_sz, mask_sz, per_slot);

    // 4) вычисляем число слотов
    size_t slots_calc = usable_mem / per_slot;
    int slots = (int)slots_calc;
    printf("[calc_slots] raw calculation slots = %zu\n", slots_calc);
    if (slots < 1) slots = 1;
    if (slots > MAX_SLOTS) slots = MAX_SLOTS;
    printf("[calc_slots] final slots (clamped 1..%d) = %d\n",
           MAX_SLOTS, slots);
    return slots;
}


/**
  * calculate_workers
  * ------------------
  * Возвращает оптимальное число рабочих потоков на основе CPU.
  */
static inline int calculate_workers(void) {
    long cores = 1;
#if defined(PLATFORM_LINUX) || defined(PLATFORM_MACOS)
    cores = sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(PLATFORM_WINDOWS)
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    cores = info.dwNumberOfProcessors;
#endif
    int w = (cores > 4) ? (int)cores - 3 : 1;
    if (w < 1) w = 1;
    if (w > MAX_WORKERS) w = MAX_WORKERS;
    printf("[calc_workers] cores=%ld, workers=%d\n", cores, w);
    return w;
}


/**
 * parse_args
 * -----------
 * Разбирает аргументы командной строки:
 *   --slots=N   — число слотов
 *   --workers=M — число воркеров
 * Возвращает:
 *   0, если оба не заданы (авто-расчёт);
 *   >0 для слотов, и workers в out_workers==0 означает авто;
 *   <0 при ошибке.
 */
static inline int parse_args(int argc, char **argv,
                             int *out_slots, int *out_workers) {
    *out_slots = 0;
    *out_workers = 0;
    for (int i = 1; i < argc; ++i) {
        if (strncmp(argv[i], "--slots=", 8) == 0) {
            int v = atoi(argv[i] + 8);
            if (v < 1 || v > MAX_SLOTS) {
                fprintf(stderr, "Error: slots must be 1..%d, got %d\n", MAX_SLOTS, v);
                return -1;
            }
            *out_slots = v;
        } else if (strncmp(argv[i], "--workers=", 10) == 0) {
            int m = atoi(argv[i] + 10);
            if (m < 1 || m > MAX_WORKERS) {
                fprintf(stderr, "Error: workers must be 1..%d, got %d\n", MAX_WORKERS, m);
                return -1;
            }
            *out_workers = m;
        } else {
            fprintf(stderr, "Error: unknown option '%s'\n", argv[i]);
            return -1;
        }
    }
    return 0;
}

#endif /* COMMON_H */
