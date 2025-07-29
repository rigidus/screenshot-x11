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

int sched_yield(void) {
    SwitchToThread();
    return 0;
}


/* ========== Windows квантизация ========== */

void platform_quantize_bgr_to_rgb332(const uint8_t *bgr_data, uint8_t *quant_data,
                                      int width, int height, int padded_width) {

    // SIMD detection (AVX2)
    static bool inited = false;
    static bool use256;
    if (!inited) {
        use256 = cpu_has_avx2();
        inited = true;
        printf("[init] AVX2 support for BGR quantization: %s\n", use256 ? "yes" : "no");
    }

    for (int y = 0; y < height; ++y) {
        // Получаем указатель на исходную строку BGR и выходную строку RGB332
        const uint8_t *row_src = bgr_data + (size_t)y * width * 3;
        uint8_t *row_q = quant_data + (size_t)y * padded_width;
        int x = 0;

#if defined(__AVX2__) && (defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86))
        // --- AVX2 ускорение ---
        // Обрабатываем блоки по 8 пикселей (24 байта BGR -> 32 байта BGRX)
        // Для каждого блока:
        // 1. Копируем 8 BGR-пикселей во временный буфер BGRX (добавляем фиктивный байт X)
        // 2. Загружаем буфер в AVX2-регистр
        // 3. Извлекаем компоненты B, G, R через маски
        // 4. Квантование: R3 = R>>5, G3 = G>>5, B2 = B>>6
        // 5. Собираем RGB332: (R3<<5)|(G3<<2)|B2
        // 6. Упаковываем результат в 8 байт и сохраняем в выходной буфер
        int lim8 = (width/8)*8;
        for (; x < lim8; x += 8) {
            uint8_t bgrx[32];
            for (int i = 0; i < 8; ++i) {
                bgrx[i*4+0] = row_src[(x+i)*3+0]; // B
                bgrx[i*4+1] = row_src[(x+i)*3+1]; // G
                bgrx[i*4+2] = row_src[(x+i)*3+2]; // R
                bgrx[i*4+3] = 0; // X (фиктивный)
            }
            __m256i pix = _mm256_loadu_si256((__m256i*)bgrx);
            // Маски для извлечения компонент
            __m256i maskB = _mm256_set1_epi32(0x000000FF);
            __m256i maskG = _mm256_set1_epi32(0x0000FF00);
            __m256i maskR = _mm256_set1_epi32(0x00FF0000);
            // Извлекаем компоненты
            __m256i B = _mm256_and_si256(pix, maskB);
            __m256i G = _mm256_srli_epi32(_mm256_and_si256(pix, maskG), 8);
            __m256i R = _mm256_srli_epi32(_mm256_and_si256(pix, maskR), 16);
            // Квантование
            __m256i R3 = _mm256_srli_epi32(R, 5);
            __m256i G3 = _mm256_srli_epi32(G, 5);
            __m256i B2 = _mm256_srli_epi32(B, 6);
            // Собираем RGB332
            __m256i rgb332 = _mm256_or_si256(_mm256_or_si256(_mm256_slli_epi32(R3,5), _mm256_slli_epi32(G3,2)), B2);
            // Упаковываем 8 байт результата
            __m128i lo = _mm256_castsi256_si128(rgb332);
            __m128i hi = _mm256_extracti128_si256(rgb332, 1);
            __m128i p16 = _mm_packus_epi32(lo, hi); // 8 x uint16_t
            __m128i p8  = _mm_packus_epi16(p16, _mm_setzero_si128()); // 8 x uint8_t
            // Сохраняем все 8 байт результата в выходной буфер
            memcpy(row_q + x, &p8, 8);
        }
#endif

        // --- Скалярная обработка ---
        // Обрабатываем оставшиеся пиксели (если ширина не кратна 8)
        for (; x < width; ++x) {
            uint8_t B = row_src[x*3+0]; // B
            uint8_t G = row_src[x*3+1]; // G
            uint8_t R = row_src[x*3+2]; // R
            // Квантование
            uint8_t R3 = R >> 5, G3 = G >> 5, B2 = B >> 6;
            row_q[x] = (uint8_t)((R3<<5)|(G3<<2)|B2);
        }
        // --- Паддинг ---
        // Заполняем оставшиеся байты строки нулями
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

bool platform_init(GlobalContext *ctx, int slots_arg) {
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

    // Если кол-во слотов не определено, вычисляем их автоматически
    if (slots_arg > 0) {
        ctx->slots = slots_arg;
    } else {
        ctx->slots = calculate_slots(ctx);
    }

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
