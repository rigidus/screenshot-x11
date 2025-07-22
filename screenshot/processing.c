#include "common.h"

#ifdef PLATFORM_LINUX
    #include <pthread.h>
    #include <sched.h>
    #include <unistd.h>
    #include <sys/stat.h>
    #include <sys/mman.h>
    #include <fcntl.h>
#endif

/* ========== Шаблоны символов ========== */
CharTemplate templates[] = {
    // Простые шаблоны для демонстрации - в реальном использовании нужно заполнить битовые маски
    {'A', {0}},
    {'B', {0}},
    {'C', {0}},
    // ... добавить реальные шаблоны
};
int n_templates = sizeof(templates) / sizeof(templates[0]);

/* ========== SIMD оптимизации ========== */

// SIMD‐проверка однородности 32-байтного региона
bool block_uniform_avx2(const uint8_t *quant, int pw, int sy, int sx) {
    uint8_t base = quant[sy*pw + sx];
    if (base == 0) return true;  // весь блок из нулей

#if defined(__AVX2__) && (defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86))
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
#else
    // Fallback скалярная версия
    for (int dy = 0; dy < BS; ++dy) {
        for (int dx = 0; dx < BS; ++dx) {
            uint8_t val = quant[(sy + dy)*pw + (sx + dx)];
            if (val != base && val != 0) return false;
        }
    }
    return true;
#endif
}

/* ========== Детекция связных регионов ========== */

OcrRegion* detect_regions(const uint8_t *mask,
                       const uint8_t *block_color,
                       int block_rows, int block_cols,
                       int *out_region_n) {
    int mask_bytes = (BS * BS + 7) / 8;
    int total_pix  = block_rows * block_cols * BS * BS;
    bool *visited  = calloc(total_pix, sizeof(bool));
    Pixel *queue   = malloc(total_pix * sizeof(Pixel));

    OcrRegion *regions = NULL;
    int region_n = 0, region_cap = 0;

    for (int by = 0; by < block_rows; ++by) {
        for (int bx = 0; bx < block_cols; ++bx) {
            int bidx = by * block_cols + bx;
            if (block_color[bidx] != MIXED) continue;

            const uint8_t *mask_block = mask + bidx * mask_bytes;
            for (int dy = 0; dy < BS; ++dy) {
                for (int dx = 0; dx < BS; ++dx) {
                    int bit   = dy * BS + dx;
                    if (!(mask_block[bit>>3] & (1 << (bit&7)))) continue;

                    int local = (bidx * BS + dy) * BS + dx;
                    if (visited[local]) continue;

                    // start BFS
                    int qh = 0, qt = 0;
                    queue[qt++] = (Pixel){(uint16_t)by,(uint16_t)bx,(uint8_t)dy,(uint8_t)dx};
                    visited[local] = true;

                    if (region_n == region_cap) {
                        region_cap = region_cap ? region_cap*2 : 8;
                        regions = realloc(regions, region_cap * sizeof(OcrRegion));
                    }
                    OcrRegion *reg = &regions[region_n++];
                    reg->count  = 0;
                    reg->pixels = NULL;
                    reg->neighbor = NULL;
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
        }
    }

    free(queue);
    free(visited);
    *out_region_n = region_n;
    return regions;
}

/* ========== Группировка регионов ========== */

static int cmp_region_miny(const void *a, const void *b) {
    const OcrRegion *r1 = *(OcrRegion**)a;
    const OcrRegion *r2 = *(OcrRegion**)b;
    return r1->miny - r2->miny;
}

static int cmp_region_minx(const void *a, const void *b) {
    const OcrRegion *r1 = *(OcrRegion**)a;
    const OcrRegion *r2 = *(OcrRegion**)b;
    return r1->minx - r2->minx;
}

void group_regions(OcrRegion *regions, int region_n,
                   Line **out_lines, int *out_line_n) {
    if (region_n == 0) { *out_lines = NULL; *out_line_n = 0; return; }

    // Собираем массив указателей и сортируем по miny
    OcrRegion **arr = malloc(region_n * sizeof(OcrRegion*));
    for (int i = 0; i < region_n; i++) arr[i] = &regions[i];
    qsort(arr, region_n, sizeof(OcrRegion*), cmp_region_miny);

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
        OcrRegion *r = arr[i];
        if (line_n == 0) {
            lines = realloc(lines, sizeof(Line) * (line_n+1));
            lines[line_n].regions = malloc(sizeof(OcrRegion*));
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
                L->regions = realloc(L->regions, sizeof(OcrRegion*)*(L->count+1));
                L->regions[L->count++] = r;
            } else {
                // новая строка
                lines = realloc(lines, sizeof(Line) * (line_n+1));
                lines[line_n].regions = malloc(sizeof(OcrRegion*));
                lines[line_n].regions[0] = r;
                lines[line_n].count = 1;
                line_n++;
            }
        }
    }

    // внутри каждой строки сортируем по X и вычисляем bbox
    for (int i = 0; i < line_n; i++) {
        Line *L = &lines[i];
        qsort(L->regions, L->count, sizeof(OcrRegion*), cmp_region_minx);
        // init bbox
        L->minx = L->regions[0]->minx;
        L->maxx = L->regions[0]->maxx;
        L->miny = L->regions[0]->miny;
        L->maxy = L->regions[0]->maxy;
        for (int j = 1; j < L->count; j++) {
            OcrRegion *r = L->regions[j];
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

void set_region_neighbors(OcrRegion *regions, int region_n) {
    // Сброс всех ссылок
    for (int i = 0; i < region_n; i++) {
        regions[i].neighbor = NULL;
    }

    for (int i = 0; i < region_n; i++) {
        OcrRegion *ri = &regions[i];
        int wi = ri->maxx - ri->minx + 1;
        int hi = ri->maxy - ri->miny + 1;
        // только «маленькие» регионы
        if (wi >= BS || hi >= BS) continue;

        int best_dx = INT_MAX, best_j = -1;
        int cy_i    = (ri->miny + ri->maxy) / 2;

        for (int j = 0; j < region_n; j++) {
            if (i == j) continue;
            OcrRegion *rj = &regions[j];
            int wj = rj->maxx - rj->minx + 1;
            int hj = rj->maxy - rj->miny + 1;
            if (wj >= BS || hj >= BS) continue;

            int dx = rj->minx - ri->maxx;
            // справа, не дальше, чем MAX_NEIGHBOR_GAP, и лучше, чем предыдущий
            if (dx <= 0 || dx > MAX_NEIGHBOR_GAP || dx >= best_dx) continue;

            // правильно: сравниваем по высоте, а не по ширине
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

/* ========== Распознавание текста ========== */

void render_region(const OcrRegion *reg, uint8_t *sample) {
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

char recognize_region(const OcrRegion *reg) {
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

/* ========== Функция анализа блоков ========== */

void analyze_blocks(FrameSlot *slot, GlobalContext *ctx) {
    const int W  = ctx->w;
    const int H  = ctx->h;
    const int pw = ctx->padded_w;
    const int bc = ctx->block_cols;
    const int br = ctx->block_rows;
    const size_t mask_sz = ((size_t)BS * BS + 7) / 8;

    // Анализ блоков 32×32 → bg/fg/mask
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {
            int idx = by*bc + bx;
            uint8_t *bg = &slot->bg[idx];
            uint8_t *fg = &slot->fg[idx];
            uint8_t *m  = slot->mask + (size_t)idx*mask_sz;

            // собираем гистограмму
            int hist[256] = {0};
            int x0 = bx * BS;
            int y0 = by * BS;
            int w_blk = MIN(BS, W - x0);
            int h_blk = MIN(BS, H - y0);
            int total = w_blk * h_blk;

            for (int dy = 0; dy < BS; ++dy) {
                for (int dx = 0; dx < BS; ++dx) {
                    uint8_t q = slot->quant[(by*BS+dy)*pw + (bx*BS+dx)];
                    hist[q]++;
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
                // uniform — очистим маску
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
