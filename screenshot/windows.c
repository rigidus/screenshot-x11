#ifdef PLATFORM_WINDOWS

/*
 * Windows-специфичная реализация xcape_pipe
 * 
 * Необходимые DLL для запуска:
 * - libpng16-16.dll  (работа с PNG изображениями)
 * - libwinpthread-1.dll (POSIX threads для MinGW)
 * - zlib1.dll        (сжатие для libpng)
 */

#include "common.h"
#include <windows.h>
#include <wingdi.h>

#ifndef _SC_NPROCESSORS_ONLN
#define _SC_NPROCESSORS_ONLN 4
#endif

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

/* ========== Windows типы и определения ========== */
typedef void* pthread_t;
typedef struct { int dummy; } pthread_attr_t;

/* ========== Windows платформо-зависимые данные ========== */
typedef struct {
    HDC                 hdc_screen;
    HDC                 hdc_mem[MAX_SLOTS];
    HBITMAP             hbitmap[MAX_SLOTS];
    BITMAPINFO          bmi;
    uint8_t            *bitmap_data[MAX_SLOTS];
} WindowsPlatformData;

/* ========== Windows реализация POSIX функций ========== */

typedef struct {
    void *(*start_routine)(void*);
    void *arg;
} thread_data_t;

static DWORD WINAPI thread_wrapper(LPVOID lpParam) {
    thread_data_t *data = (thread_data_t*)lpParam;
    data->start_routine(data->arg);
    free(data);
    return 0;
}

int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine)(void*), void *arg) {
    (void)attr;
    thread_data_t *data = malloc(sizeof(thread_data_t));
    if (!data) return -1;
    
    data->start_routine = start_routine;
    data->arg = arg;
    
    HANDLE h = CreateThread(NULL, 0, thread_wrapper, data, 0, NULL);
    *thread = (void*)h;  // Приводим HANDLE к void*
    return (h == NULL) ? -1 : 0;
}

int pthread_join(pthread_t thread, void **retval) {
    (void)retval;
    HANDLE h = (HANDLE)thread;  // Приводим void* обратно к HANDLE
    WaitForSingleObject(h, INFINITE);
    CloseHandle(h);
    return 0;
}

long sysconf(int name) {
    if (name == _SC_NPROCESSORS_ONLN) {
        SYSTEM_INFO si;
        GetSystemInfo(&si);
        return si.dwNumberOfProcessors;
    }
    return -1;
}

int sched_yield(void) {
    SwitchToThread();
    return 0;
}


/* ========== Windows квантизация ========== */

void platform_quantize_bgr_to_rgb332(const uint8_t *bgr_data, uint8_t *quant_data, 
                                      int width, int height, int padded_width) {
    // Однократный детект SIMD возможностей
    static bool inited = false;
    static bool use256;
    if (!inited) {
        use256 = cpu_has_avx2();
        inited = true;
        printf("[init] AVX2 support for BGR quantization: %s\n", use256 ? "yes" : "no");
    }

    // Квантование BGR → RGB332
    for (int y = 0; y < height; ++y) {
        const uint8_t *row_src = bgr_data + (size_t)y * width * 3;  // BGR (3 байта на пиксель)
        uint8_t *row_q = quant_data + (size_t)y * padded_width;
        int x = 0;

        // Скалярная версия (BGR)
        for (; x < width; ++x) {
            uint8_t B = row_src[x*3+0];  // BGR
            uint8_t G = row_src[x*3+1];
            uint8_t R = row_src[x*3+2];
            uint8_t R3 = R >> 5, G3 = G >> 5, B2 = B >> 6;
            row_q[x] = (uint8_t)((R3<<5)|(G3<<2)|B2);
        }
        
        // Паддинг
        for (; x < padded_width; ++x) {
            row_q[x] = 0;
        }
    }

    // Заполняем padding строки
    int block_rows = (height + BS - 1) / BS;
    int padded_height = block_rows * BS;
    for (int y = height; y < padded_height; ++y) {
        uint8_t *row_q = quant_data + (size_t)y * padded_width;
        memset(row_q, 0, padded_width);
    }

#ifdef __SSE2__
    _mm_sfence();  // дождаться store
#endif
}

/* ========== Реализация платформо-зависимых функций ========== */

bool platform_init(GlobalContext *ctx) {
    WindowsPlatformData *pdata = malloc(sizeof(WindowsPlatformData));
    if (!pdata) return false;
    
    ctx->platform_data = pdata;
    
    // Получаем контекст экрана
    pdata->hdc_screen = GetDC(NULL);
    if (!pdata->hdc_screen) {
        free(pdata);
        return false;
    }

    // Получаем размеры экрана
    ctx->w = GetSystemMetrics(SM_CXSCREEN);
    ctx->h = GetSystemMetrics(SM_CYSCREEN);

    // Настраиваем BITMAPINFO для 24-битного RGB
    memset(&pdata->bmi, 0, sizeof(pdata->bmi));
    pdata->bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    pdata->bmi.bmiHeader.biWidth = ctx->w;
    pdata->bmi.bmiHeader.biHeight = -ctx->h; // отрицательная высота для top-down DIB
    pdata->bmi.bmiHeader.biPlanes = 1;
    pdata->bmi.bmiHeader.biBitCount = 24; // 24 бита на пиксель (RGB)
    pdata->bmi.bmiHeader.biCompression = BI_RGB;

    // Создаем контексты памяти и битмапы для каждого слота
    for (uint32_t i = 0; i < ctx->slots; ++i) {
        pdata->hdc_mem[i] = CreateCompatibleDC(pdata->hdc_screen);
        if (!pdata->hdc_mem[i]) {
            // Очистка уже созданных ресурсов
            for (uint32_t j = 0; j < i; ++j) {
                if (pdata->hbitmap[j]) DeleteObject(pdata->hbitmap[j]);
                if (pdata->hdc_mem[j]) DeleteDC(pdata->hdc_mem[j]);
            }
            ReleaseDC(NULL, pdata->hdc_screen);
            free(pdata);
            return false;
        }

        pdata->hbitmap[i] = CreateDIBSection(pdata->hdc_mem[i], &pdata->bmi, DIB_RGB_COLORS, 
                                           (void**)&pdata->bitmap_data[i], NULL, 0);
        if (!pdata->hbitmap[i]) {
            DeleteDC(pdata->hdc_mem[i]);
            // Очистка других ресурсов...
            for (uint32_t j = 0; j < i; ++j) {
                if (pdata->hbitmap[j]) DeleteObject(pdata->hbitmap[j]);
                if (pdata->hdc_mem[j]) DeleteDC(pdata->hdc_mem[j]);
            }
            ReleaseDC(NULL, pdata->hdc_screen);
            free(pdata);
            return false;
        }

        SelectObject(pdata->hdc_mem[i], pdata->hbitmap[i]);
    }

    printf("[init] Windows GDI %dx%d\n", ctx->w, ctx->h);
    return true;
}

void platform_cleanup(GlobalContext *ctx) {
    if (!ctx->platform_data) return;
    
    WindowsPlatformData *pdata = (WindowsPlatformData*)ctx->platform_data;
    
    for (uint32_t i = 0; i < ctx->slots; ++i) {
        if (pdata->hbitmap[i]) {
            DeleteObject(pdata->hbitmap[i]);
        }
        if (pdata->hdc_mem[i]) {
            DeleteDC(pdata->hdc_mem[i]);
        }
    }
    
    if (pdata->hdc_screen) {
        ReleaseDC(NULL, pdata->hdc_screen);
    }
    
    free(pdata);
    ctx->platform_data = NULL;
}

bool platform_capture_screen(GlobalContext *ctx, int slot_index) {
    WindowsPlatformData *pdata = (WindowsPlatformData*)ctx->platform_data;
    
    // Захватываем изображение через BitBlt в DIB секцию
    if (!BitBlt(pdata->hdc_mem[slot_index], 0, 0, ctx->w, ctx->h, 
                pdata->hdc_screen, 0, 0, SRCCOPY)) {
        return false;
    }
    
    // Устанавливаем указатель на raw данные для отладки
    ctx->slot[slot_index].raw = pdata->bitmap_data[slot_index];
    
    // Прямая квантизация BGR → RGB332
    platform_quantize_bgr_to_rgb332(pdata->bitmap_data[slot_index], 
                                     ctx->slot[slot_index].quant,
                                     ctx->w, ctx->h, ctx->padded_w);
    
    return true;
}

#endif
