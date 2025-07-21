#include "common.h"

/* ========== Вспомогательные функции путей ========== */

void create_screenshot_path(char *full_path, size_t path_size, const char *filename) {
    #ifdef PLATFORM_WINDOWS
        // По умолчанию сохраняем в C:\Tmp
        const char *screenshot_dir = "C:\\Tmp";
        snprintf(full_path, path_size, "%s\\%s", screenshot_dir, filename);
    #else
        snprintf(full_path, path_size, "%s%s", SCREENSHOT_PATH, filename);
    #endif
}

/* ========== Отладочные функции ========== */

void dump_png_rgb(const char *fname, int W, int H, const uint8_t *rgb) {
    // Создаем полный путь к файлу
    char full_path[MAX_SCREENSHOT_PATH];
    create_screenshot_path(full_path, sizeof(full_path), fname);
    
    // Конвертируем PNG в BMP для совместимости
    char bmp_fname[MAX_SCREENSHOT_PATH];
    strcpy(bmp_fname, full_path);
    char *ext = strstr(bmp_fname, ".png");
    if (ext) strcpy(ext, ".bmp");

    // Конвертируем RGB в RGBA и сохраняем как BMP
    uint8_t *rgba = malloc((size_t)W * H * 4);
    if (!rgba) {
        printf("[error] Failed to allocate memory for RGBA conversion\n");
        return;
    }

    for (int i = 0; i < W * H; i++) {
        rgba[i*4+0] = rgb[i*3+0]; // R
        rgba[i*4+1] = rgb[i*3+1]; // G
        rgba[i*4+2] = rgb[i*3+2]; // B
        rgba[i*4+3] = 255;        // A
    }

    printf("[debug] Saving screenshot: %s\n", bmp_fname);
    dump_rgba_as_bmp(bmp_fname, W, H, rgba);
    free(rgba);
}


// Функция для сохранения RGBA данных как BMP
void dump_rgba_as_bmp(const char *fname, int W, int H, const uint8_t *rgba) {
    if (!rgba) {
        printf("[error] RGBA data is NULL!\n");
        return;
    }

    FILE *fp = fopen(fname, "wb");
    if (!fp) {
        perror("dump_rgba_as_bmp: fopen");
        return;
    }

    // BMP заголовок для 24-битного изображения
    uint32_t file_size = 54 + W * H * 3;
    uint32_t data_offset = 54;
    uint32_t header_size = 40;
    uint16_t planes = 1;
    uint16_t bits_per_pixel = 24;

    // BMP File Header
    fwrite("BM", 1, 2, fp);  // Signature
    fwrite(&file_size, 4, 1, fp);
    fwrite("\0\0\0\0", 4, 1, fp);  // Reserved
    fwrite(&data_offset, 4, 1, fp);

    // BMP Info Header
    fwrite(&header_size, 4, 1, fp);
    fwrite(&W, 4, 1, fp);
    fwrite(&H, 4, 1, fp);
    fwrite(&planes, 2, 1, fp);
    fwrite(&bits_per_pixel, 2, 1, fp);
    fwrite("\0\0\0\0", 4, 1, fp);  // Compression
    fwrite("\0\0\0\0", 4, 1, fp);  // Image size
    fwrite("\0\0\0\0", 4, 1, fp);  // X pixels per meter
    fwrite("\0\0\0\0", 4, 1, fp);  // Y pixels per meter
    fwrite("\0\0\0\0", 4, 1, fp);  // Colors used
    fwrite("\0\0\0\0", 4, 1, fp);  // Colors important

    // Данные изображения (BMP хранит строки снизу вверх)
    // Конвертируем RGBA в BGR для BMP
    for (int y = H - 1; y >= 0; y--) {
        for (int x = 0; x < W; x++) {
            const uint8_t *pixel = rgba + (y * W + x) * 4;
            uint8_t bgr[3] = { pixel[2], pixel[1], pixel[0] };  // RGBA -> BGR
            fwrite(bgr, 3, 1, fp);
        }
    }

    fclose(fp);
}

// расширение RGB332 → [0..255]
static inline void expand_rgb332(uint8_t q, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t r3 = (q >> 5) & 0x7;
    uint8_t g3 = (q >> 2) & 0x7;
    uint8_t b2 =  q        & 0x3;
    *r = expand3(r3);
    *g = expand3(g3);
    *b = expand2(b2);
}

// Debug: dump quantized RGB332 buffer as RGB888 PNG/BMP
void debug_dump_quant(int slot, const uint8_t *quant, int padded_w, GlobalContext *ctx) {
    uint8_t *rgb = malloc((size_t)ctx->w * ctx->h * 3);
    if (!rgb) {
        printf("debug_dump_quant: malloc failed\n");
        return;
    }

    for (int y = 0; y < ctx->h; ++y) {
        for (int x = 0; x < ctx->w; ++x) {
            uint8_t q = quant[y * padded_w + x];
            uint8_t r, g, b;
            expand_rgb332(q, &r, &g, &b);
            size_t idx = (size_t)y * ctx->w + x;
            rgb[idx*3+0] = r;
            rgb[idx*3+1] = g;
            rgb[idx*3+2] = b;
        }
    }

    char fn[256];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "dbg_quant_%d_%ld_%09ld.png",
             slot, ts.tv_sec, ts.tv_nsec);
    dump_png_rgb(fn, ctx->w, ctx->h, rgb);
    free(rgb);
}

// Рисует отладочное изображение, где каждый 32×32-блок заливается соответствующими цветами
static void dump_filled_blocks_png(const char *fname, const FrameSlot *slot, GlobalContext *ctx) {
    const int W   = ctx->w;
    const int H   = ctx->h;
    const int bc  = ctx->block_cols;
    const int br  = ctx->block_rows;
    const size_t mask_sz = ((size_t)BS*BS + 7)/8;

    // 1) Серый фон
    uint8_t *rgb = malloc((size_t)W * H * 3);
    if (!rgb) {
        printf("dump_filled_blocks_png: malloc failed\n");
        return;
    }
    memset(rgb, 127, (size_t)W * H * 3);

    // 2) По блокам 32×32
    for (int by = 0; by < br; ++by) {
        for (int bx = 0; bx < bc; ++bx) {
            int idx = by*bc + bx;
            uint8_t bgc = slot->bg[idx];
            uint8_t fgc = slot->fg[idx];
            uint8_t *m   = slot->mask + (size_t)idx * mask_sz;

            // распаковать 2 цвета в RGB888
            uint8_t br8, bg8, bb8, fr8, fg8, fb8;
            expand_rgb332(bgc, &br8, &bg8, &bb8);
            expand_rgb332(fgc, &fr8, &fg8, &fb8);

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
                // рамки и диагонали
                // верхняя белая
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = (size_t)y0 * W + (x0+dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                }
                // левая белая
                for (int dy = 0; dy < h_blk; ++dy) {
                    size_t p = (size_t)(y0+dy) * W + x0;
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 255;
                }
                // нижняя чёрная
                for (int dx = 0; dx < w_blk; ++dx) {
                    size_t p = (size_t)(y0 + h_blk - 1) * W + (x0+dx);
                    rgb[p*3+0] = rgb[p*3+1] = rgb[p*3+2] = 0;
                }
                // правая чёрная
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
                        size_t p = (size_t)(y0+dy)*W + (x0 + w_blk - 1 - dx);
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

void debug_dump_filled(int slot_idx, const FrameSlot *slot, GlobalContext *ctx) {
    char fname[256];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fname, sizeof(fname), "dbg_fill_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_filled_blocks_png(fname, slot, ctx);
}

void debug_dump_regions(int slot_idx, const OcrRegion *regions, int region_n, GlobalContext *ctx) {
    uint8_t *rgb = malloc((size_t)ctx->w * ctx->h * 3);
    if (!rgb) {
        printf("debug_dump_regions: malloc failed\n");
        return;
    }
    memset(rgb, 255, (size_t)ctx->w * ctx->h * 3);

    // draw black pixels for regions
    for (int r = 0; r < region_n; ++r) {
        for (int i = 0; i < regions[r].count; ++i) {
            Pixel p = regions[r].pixels[i];
            int x = p.bx*BS + p.dx;
            int y = p.by*BS + p.dy;
            if (x >= 0 && x < ctx->w && y >= 0 && y < ctx->h) {
                size_t off = ((size_t)y * ctx->w + x) * 3;
                rgb[off+0] = rgb[off+1] = rgb[off+2] = 0;
            }
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

        // горизонтальные линии
        for (int x = MAX(0, minx); x <= MIN(ctx->w-1, maxx); ++x) {
            if (miny >= 0 && miny < ctx->h) {
                size_t t = ((size_t)miny * ctx->w + x) * 3;
                rgb[t+0]=rr; rgb[t+1]=gg; rgb[t+2]=bb;
            }
            if (maxy >= 0 && maxy < ctx->h) {
                size_t b = ((size_t)maxy * ctx->w + x) * 3;
                rgb[b+0]=rr; rgb[b+1]=gg; rgb[b+2]=bb;
            }
        }
        // вертикальные линии
        for (int y = MAX(0, miny); y <= MIN(ctx->h-1, maxy); ++y) {
            if (minx >= 0 && minx < ctx->w) {
                size_t l = ((size_t)y * ctx->w + minx) * 3;
                rgb[l+0]=rr; rgb[l+1]=gg; rgb[l+2]=bb;
            }
            if (maxx >= 0 && maxx < ctx->w) {
                size_t rgt = ((size_t)y * ctx->w + maxx) * 3;
                rgb[rgt+0]=rr; rgb[rgt+1]=gg; rgb[rgt+2]=bb;
            }
        }
    }

    char fn[256];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "regions_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_png_rgb(fn, ctx->w, ctx->h, rgb);
    free(rgb);
}

void debug_recognize(int slot_idx, OcrRegion *regions, int region_n) {
    for (int r = 0; r < region_n; ++r) {
        char c = recognize_region(&regions[r]);
        printf("slot %d, region %d: '%c'\n", slot_idx, r, c);
    }
}

void debug_dump_lines(int slot_idx, Line *lines, int line_n, GlobalContext *ctx) {
    uint8_t *rgb = malloc((size_t)ctx->w * ctx->h * 3);
    if (!rgb) {
        printf("debug_dump_lines: malloc failed\n");
        return;
    }
    memset(rgb, 255, (size_t)ctx->w * ctx->h * 3);

    srand(slot_idx);
    for (int i = 0; i < line_n; i++) {
        Line *L = &lines[i];
        uint8_t rr = rand() & 255;
        uint8_t gg = rand() & 255;
        uint8_t bb = rand() & 255;

        // по периметру bbox
        for (int x = MAX(0, L->minx); x <= MIN(ctx->w-1, L->maxx); x++) {
            if (L->miny >= 0 && L->miny < ctx->h) {
                size_t t = ((size_t)L->miny * ctx->w + x)*3;
                rgb[t+0]=rr; rgb[t+1]=gg; rgb[t+2]=bb;
            }
            if (L->maxy >= 0 && L->maxy < ctx->h) {
                size_t b = ((size_t)L->maxy * ctx->w + x)*3;
                rgb[b+0]=rr; rgb[b+1]=gg; rgb[b+2]=bb;
            }
        }
        for (int y = MAX(0, L->miny); y <= MIN(ctx->h-1, L->maxy); y++) {
            if (L->minx >= 0 && L->minx < ctx->w) {
                size_t l = ((size_t)y * ctx->w + L->minx)*3;
                rgb[l+0]=rr; rgb[l+1]=gg; rgb[l+2]=bb;
            }
            if (L->maxx >= 0 && L->maxx < ctx->w) {
                size_t r = ((size_t)y * ctx->w + L->maxx)*3;
                rgb[r+0]=rr; rgb[r+1]=gg; rgb[r+2]=bb;
            }
        }
    }

    char fn[256];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "lines_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_png_rgb(fn, ctx->w, ctx->h, rgb);
    free(rgb);
}

void debug_dump_chains(int slot_idx, OcrRegion *regions, int region_n, GlobalContext *ctx) {
    uint8_t *rgb = malloc((size_t)ctx->w * ctx->h * 3);
    if (!rgb) {
        printf("debug_dump_chains: malloc failed\n");
        return;
    }
    memset(rgb, 255, (size_t)ctx->w * ctx->h * 3);

    // 1) Рисуем чёрные рамки вокруг каждого маленького региона
    for (int i = 0; i < region_n; i++) {
        OcrRegion *r = &regions[i];
        int w = r->maxx - r->minx + 1;
        int h = r->maxy - r->miny + 1;
        if (w < 32 && h < 32) {
            for (int x = MAX(0, r->minx); x <= MIN(ctx->w-1, r->maxx); x++) {
                if (r->miny >= 0 && r->miny < ctx->h) {
                    size_t t = ((size_t)r->miny * ctx->w + x)*3;
                    rgb[t+0] = rgb[t+1] = rgb[t+2] = 0;
                }
                if (r->maxy >= 0 && r->maxy < ctx->h) {
                    size_t b = ((size_t)r->maxy * ctx->w + x)*3;
                    rgb[b+0] = rgb[b+1] = rgb[b+2] = 0;
                }
            }
            for (int y = MAX(0, r->miny); y <= MIN(ctx->h-1, r->maxy); y++) {
                if (r->minx >= 0 && r->minx < ctx->w) {
                    size_t l = ((size_t)y * ctx->w + r->minx)*3;
                    rgb[l+0] = rgb[l+1] = rgb[l+2] = 0;
                }
                if (r->maxx >= 0 && r->maxx < ctx->w) {
                    size_t rr= ((size_t)y * ctx->w + r->maxx)*3;
                    rgb[rr+0]= rgb[rr+1]= rgb[rr+2]= 0;
                }
            }
        }
    }

    // 2) Помечаем все регионы, на которые кто-то ссылается как на neighbor
    bool *is_target = calloc(region_n, sizeof(bool));
    for (int i = 0; i < region_n; i++) {
        OcrRegion *nbr = regions[i].neighbor;
        if (nbr) {
            int idx = (int)(nbr - regions);
            if (idx >= 0 && idx < region_n) {
                is_target[idx] = true;
            }
        }
    }

    // 3) Для каждого «головы» цепочки рисуем красную рамку вокруг общего bbox
    srand(slot_idx + 123);
    for (int i = 0; i < region_n; i++) {
        OcrRegion *r0 = &regions[i];
        if (!is_target[i] && r0->neighbor) {
            int minx = r0->minx, miny = r0->miny;
            int maxx = r0->maxx, maxy = r0->maxy;
            // обходим цепочку
            for (OcrRegion *cur = r0; cur; cur = cur->neighbor) {
                if (cur->minx < minx) minx = cur->minx;
                if (cur->miny < miny) miny = cur->miny;
                if (cur->maxx > maxx) maxx = cur->maxx;
                if (cur->maxy > maxy) maxy = cur->maxy;
            }

            // рисуем красную рамку для всей строки
            for (int x = MAX(0, minx); x <= MIN(ctx->w-1, maxx); x++) {
                if (miny >= 0 && miny < ctx->h) {
                    size_t t = ((size_t)miny * ctx->w + x)*3;
                    rgb[t+0] = 255; rgb[t+1] =   0; rgb[t+2] =   0;
                }
                if (maxy >= 0 && maxy < ctx->h) {
                    size_t b = ((size_t)maxy * ctx->w + x)*3;
                    rgb[b+0] = 255; rgb[b+1] =   0; rgb[b+2] =   0;
                }
            }
            for (int y = MAX(0, miny); y <= MIN(ctx->h-1, maxy); y++) {
                if (minx >= 0 && minx < ctx->w) {
                    size_t l = ((size_t)y * ctx->w + minx)*3;
                    rgb[l+0] = 255; rgb[l+1] =   0; rgb[l+2] =   0;
                }
                if (maxx >= 0 && maxx < ctx->w) {
                    size_t rr= ((size_t)y * ctx->w + maxx)*3;
                    rgb[rr+0]= 255; rgb[rr+1]=   0; rgb[rr+2]=   0;
                }
            }
        }
    }
    free(is_target);

    // 4) Сохраняем
    char fn[256];
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(fn, sizeof(fn), "chains_%d_%ld.png", slot_idx, ts.tv_sec);
    dump_png_rgb(fn, ctx->w, ctx->h, rgb);
    free(rgb);
}
