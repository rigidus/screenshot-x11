/*
 * glyphs.c — collect & store OCR glyph bitmaps into glifs/<WxH>/<name>.bmp
 */
#include "glyphs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef PLATFORM_WINDOWS
#  include <direct.h>
#  include <io.h>
#  define mkpath(dir)   _mkdir(dir)
#  define file_exists(path) (_access((path),0)==0)
#else
#  include <sys/stat.h>
#  include <unistd.h>
#  define mkpath(dir)   mkdir((dir),0755)
#  define file_exists(path) (access((path),F_OK)==0)
#endif

#define GLYPHS_ROOT    "glifs"
#define MAX_NAME_LEN   250

// 6-bit alphabet
static const char alphabet[] =
        "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "-_";

// ensure directory exists (ignore EEXIST)
static void ensure_dir(const char *path) {
    if (mkpath(path) != 0 && errno != EEXIST) {
        fprintf(stderr, "[glyphs] mkdir %s failed: %s\n", path, strerror(errno));
    }
}

void init_glyphs_store(void) {
    ensure_dir(GLYPHS_ROOT);
}

// generate a printable name from bits[0..w*h), 6 bits per character
static void make_glyph_name(const uint8_t *bits, int w, int h, char *out) {
    int total = w * h;
    int bitpos = 0, outpos = 0;
    while (bitpos < total && outpos < MAX_NAME_LEN) {
        unsigned val = 0;
        for (int b = 0; b < 6; ++b) {
            int idx = bitpos + b;
            int bit = 0;
            if (idx < total) {
                bit = (bits[idx>>3] >> (idx&7)) & 1;
            }
            val = (val << 1) | bit;
        }
        out[outpos++] = alphabet[val & 0x3F];
        bitpos += 6;
    }
    out[outpos] = '\0';
}

// write a 24-bit BMP (black & white) at path
static void write_bitmap_bmp(const char *path, const uint8_t *bits, int w, int h) {
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        fprintf(stderr, "[glyphs] fopen %s failed: %s\n", path, strerror(errno));
        return;
    }
    int row_stride = (3 * w + 3) & ~3;
    uint32_t file_size = 54 + row_stride * h;
    uint32_t data_offset = 54;
    uint32_t header_size = 40;
    uint16_t planes = 1, bpp = 24;

    // file header
    fwrite("BM",1,2,fp);
    fwrite(&file_size,4,1,fp);
    fwrite("\0\0\0\0",4,1,fp);
    fwrite(&data_offset,4,1,fp);

    // info header
    fwrite(&header_size,4,1,fp);
    fwrite(&w,4,1,fp);
    fwrite(&h,4,1,fp);
    fwrite(&planes,2,1,fp);
    fwrite(&bpp,2,1,fp);
    fwrite("\0\0\0\0",4,1,fp);
    fwrite("\0\0\0\0",4,1,fp);
    fwrite("\0\0\0\0",4,1,fp);
    fwrite("\0\0\0\0",4,1,fp);
    fwrite("\0\0\0\0",4,1,fp);
    fwrite("\0\0\0\0",4,1,fp);

    static const uint8_t pad[3] = {0,0,0};

    // pixel data (bottom-up)
    for (int y = h-1; y >= 0; --y) {
        for (int x = 0; x < w; ++x) {
            int bit = y*w + x;
            int v = (bits[bit>>3] >> (bit&7)) & 1;
            uint8_t r = v?0:255, g = v?0:255, b = v?0:255;
            uint8_t px[3] = { b, g, r };
            fwrite(px,3,1,fp);
        }
        int padn = row_stride - 3*w;
        if (padn) fwrite(pad, padn,1,fp);
    }

    fclose(fp);
}

void save_glyph(const uint8_t *bits, int w, int h) {
    char subdir[64];
    snprintf(subdir, sizeof(subdir), GLYPHS_ROOT "/%dx%d", w, h);
    ensure_dir(subdir);

    char name[MAX_NAME_LEN+1];
    make_glyph_name(bits, w, h, name);

    char path[512];
    snprintf(path, sizeof(path), "%s/%s.bmp", subdir, name);

    if (file_exists(path)) {
        return;
    }
    write_bitmap_bmp(path, bits, w, h);
    fprintf(stderr, "[glyphs] saved %s\n", path);
}
