/*
 * xcape_pipe.c - Модульная версия
 * ----------------------------------------------------------------------------
 * Кроссплатформенное приложение для захвата скриншотов и анализа текста
 * Поддерживает Windows (GDI) и Linux (X11/XShm)
 */

#include "common.h"

#ifdef PLATFORM_LINUX
    #include <pthread.h>
    #include <sched.h>
    #include <unistd.h>
    #include <sys/stat.h>
    #include <sys/mman.h>
    #include <fcntl.h>
#endif

#ifdef PLATFORM_WINDOWS
    // pthread функции определены в windows.c
    typedef void* pthread_t;
    typedef struct { int dummy; } pthread_attr_t;

    int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine)(void*), void *arg);
    int pthread_join(pthread_t thread, void **retval);
    long sysconf(int name);
    int sched_yield(void);
#endif

#include <math.h>
#include <limits.h>

/* ========== Глобальный контекст ========== */
static GlobalContext g;

/* ========== Общие функции обработки ========== */

void allocate_bigmem(GlobalContext *ctx) {
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

    // размер памяти одного слота
    size_t per_slot = align_up(offset_mask + mask_sz);

    // размер памяти под все слоты
    ctx->bigmem_sz = per_slot * ctx->slots;

    // Выделяем память
#ifdef PLATFORM_LINUX
    ctx->bigmem = mmap(NULL, ctx->bigmem_sz,
                       PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (ctx->bigmem == MAP_FAILED) die("mmap bigmem");
#else
    ctx->bigmem = malloc(ctx->bigmem_sz);
    if (!ctx->bigmem) die("malloc bigmem");
#endif

    // раздаём "нарезки" каждому слоту
    for (uint32_t i = 0; i < ctx->slots; ++i) {
        uint8_t *base = ctx->bigmem + i * per_slot;
        ctx->slot[i] = (FrameSlot){
            .st = FREE,
            .raw   = NULL,  // будет установлен при захвате
            .quant = base + offset_quant,
            .bg    = base + offset_bg,
            .fg    = base + offset_fg,
            .mask  = base + offset_mask,
            .block_count = ctx->block_count
        };
        printf("[slot %u] quant=%p bg=%p fg=%p mask=%p\n",
               i, ctx->slot[i].quant, ctx->slot[i].bg,
               ctx->slot[i].fg, ctx->slot[i].mask);
    }
}

/* ========== Потоки ========== */

static void *capture_thread(void *arg) {
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
            printf("[drop] slot %u busy\n", idx);
            idx = (idx + 1) % g.slots;
            continue;
        }

        // Захватываем изображение в зависимости от платформы
        if (!platform_capture_screen(&g, idx)) {
            printf("[error] Failed to capture screen for slot %u\n", idx);
            idx = (idx + 1) % g.slots;
            continue;
        }

        // Помечаем время начала обработки и переводим слот в RAW_READY
        s->t_start = now_ns();
        atomic_store_explicit(&s->st, RAW_READY, memory_order_release);
        printf("[capture] Slot %u set to RAW_READY\n", idx);

        // Переходим к следующему слоту
        idx = (idx + 1) % g.slots;
    }

    return NULL;
}

static void *worker_thread(void *arg) {
    uint32_t worker_id = (uint32_t)(uintptr_t)arg;
    printf("[worker %u] Worker thread started\n", worker_id);

    while (1) {
        bool found = false;

        for (uint32_t i = 0; i < g.slots; ++i) {
            FrameSlot *s = &g.slot[i];
            enum slot_state expected = RAW_READY;

            if (atomic_compare_exchange_strong(&s->st, &expected, IN_PROGRESS)) {
                printf("[worker %u] Processing slot %u\n", worker_id, i);

                // === Анализ uniform-блоков 32×32 ===
                analyze_blocks(s, &g);

                // Отладочные дампы (включаем для проверки)
                debug_dump_quant(i, s->quant, g.padded_w, &g);
                debug_dump_filled(i, s, &g);

                // build per-block color flags: MIXED if fg xor bg != 0
                int bc = g.block_rows * g.block_cols;
                uint8_t *block_color = malloc(bc);
                if (!block_color) {
                    printf("[worker %u] Failed to allocate block_color\n", worker_id);
                    atomic_store_explicit(&s->st, FREE, memory_order_release);
                    continue;
                }

                for (int j = 0; j < bc; ++j) {
                    block_color[j] = (s->fg[j] ^ s->bg[j]) ? MIXED : 0;
                }

                int region_n;
                OcrRegion *regions = detect_regions(s->mask, block_color,
                                                g.block_rows, g.block_cols,
                                                &region_n);

                // Фильтруем слишком большие регионы (>32×32)
                int small_n = 0;
                OcrRegion *small = malloc(region_n * sizeof(OcrRegion));
                if (small && regions) {
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
                    regions = small;
                    region_n = small_n;
                }

                if (regions) {
                    // Отладочные дампы
                    debug_dump_regions(i, regions, region_n, &g);
                    debug_recognize(i, regions, region_n);

                    // связываем «соседей» и рисуем цепочки
                    set_region_neighbors(regions, region_n);
                    debug_dump_chains(i, regions, region_n, &g);

                    // Группировка в строки
                    Line *lines;
                    int line_n;
                    group_regions(regions, region_n, &lines, &line_n);
                    if (lines) {
                        debug_dump_lines(i, lines, line_n, &g);

                        // Освобождаем память строк
                        for (int l = 0; l < line_n; l++) {
                            free(lines[l].regions);
                        }
                        free(lines);
                    }

                    // Освобождаем регионы
                    for (int r = 0; r < region_n; ++r) {
                        free(regions[r].pixels);
                    }
                    free(regions);
                }

                free(block_color);

                // Переводим в состояние готовности к сериализации
                atomic_store_explicit(&s->st, QUANT_DONE, memory_order_release);
                printf("[worker %u] Slot %u set to QUANT_DONE\n", worker_id, i);
                found = true;
                break;
            }
        }

        if (!found) {
            sched_yield();
        }
    }

    return NULL;
}

static void *serializer_thread(void *arg) {
    (void)arg;
    printf("[serializer] Serializer thread started\n");

    while (1) {
        bool found = false;

        for (uint32_t i = 0; i < g.slots; ++i) {
            FrameSlot *s = &g.slot[i];
            enum slot_state expected = QUANT_DONE;

            if (atomic_compare_exchange_strong(&s->st, &expected, SERIALIZING)) {
                printf("[serializer] Found slot %u ready for serialization\n", i);

                // Создаем имя файла для обработанного скриншота
                char bmp_filename[256];
#ifdef PLATFORM_WINDOWS
                char *temp_dir = getenv("TEMP");
                if (temp_dir) {
                    snprintf(bmp_filename, sizeof(bmp_filename), "%s\\screenshot_processed_%llu.bmp",
                             temp_dir, (unsigned long long)s->t_start);
                } else {
                    snprintf(bmp_filename, sizeof(bmp_filename), "screenshot_processed_%llu.bmp",
                             (unsigned long long)s->t_start);
                }
#else
                snprintf(bmp_filename, sizeof(bmp_filename), "/tmp/screenshot_processed_%llu.bmp",
                         (unsigned long long)s->t_start);
#endif

                // Сохраняем квантованное изображение как отладочную информацию
                printf("[serializer] Saving processed quant data for slot %u\n", i);
                debug_dump_quant(i, s->quant, g.padded_w, &g);

                printf("[serializer] Processing slot %u completed (time: %llu ns)\n",
                       i, (unsigned long long)(now_ns() - s->t_start));

                // Освобождаем слот
                atomic_store_explicit(&s->st, FREE, memory_order_release);
                printf("[serializer] Slot %u freed\n", i);
                found = true;
                break;
            }
        }

        if (!found) {
            sched_yield();
        }
    }

    return NULL;
}

/* ========== Главная функция ========== */
int main(int argc, char **argv) {
    // Парсинг аргументов --slots=N
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

    // Определяем число рабочих потоков
    int cores = get_num_cores();
    g.workers = (cores > 4) ? cores - 3 : 1;

    // Инициализация платформы для захвата скриншотов
    if (!platform_init(&g)) {
        fprintf(stderr, "Failed to initialize platform\n");
        return 1;
    }

    // Вычисление размеров блоков 32×32 и паддинга
    g.block_cols  = (g.w + BS - 1) / BS;
    g.block_rows  = (g.h + BS - 1) / BS;
    g.block_count = g.block_cols * g.block_rows;
    g.padded_w    = g.block_cols * BS;
    g.padded_h    = g.block_rows * BS;

    printf("[init] screen %dx%d, padded %dx%d, blocks %dx%d (%d total)\n",
           g.w, g.h, g.padded_w, g.padded_h,
           g.block_cols, g.block_rows, g.block_count);

    // Аллокация больших буферов
    allocate_bigmem(&g);

    // Запуск потоков: захват, обработка, сериализация
    pthread_t cap_tid, ser_tid;
    pthread_t *w_tids = calloc(g.workers, sizeof(pthread_t));

    pthread_create(&cap_tid, NULL, capture_thread, NULL);
    pthread_create(&ser_tid, NULL, serializer_thread, NULL);

    for (uint32_t i = 0; i < g.workers; ++i) {
        pthread_create(&w_tids[i], NULL, worker_thread, (void*)(uintptr_t)i);
    }

    printf("[init] Started %d worker threads\n", g.workers);

    // Ждём потоков (на самом деле capture_thread бесконечен)
    pthread_join(cap_tid, NULL);
    pthread_join(ser_tid, NULL);
    for (uint32_t i = 0; i < g.workers; ++i) {
        pthread_join(w_tids[i], NULL);
    }

    // Очистка ресурсов
    if (g.bigmem) {
#ifdef PLATFORM_LINUX
        munmap(g.bigmem, g.bigmem_sz);
#else
        free(g.bigmem);
#endif
    }
    platform_cleanup(&g);
    free(w_tids);

    return 0;
}
