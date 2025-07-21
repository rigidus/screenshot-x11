#include "common.h"
#include <CoreGraphics/CoreGraphics.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct {    
    int dummy;
} MacPlatformData;

bool platform_init(GlobalContext *ctx) {
    MacPlatformData *pdata = malloc(sizeof(MacPlatformData));
    if (!pdata) return false;
    ctx->platform_data = pdata;

    CGDirectDisplayID display = CGMainDisplayID();
    ctx->w = (int)CGDisplayPixelsWide(display);
    ctx->h = (int)CGDisplayPixelsHigh(display);

    printf("[init] macOS display %dx%d\n", ctx->w, ctx->h);
    return true;
}

void platform_cleanup(GlobalContext *ctx) {
    if (!ctx->platform_data) return;
    free(ctx->platform_data);
    ctx->platform_data = NULL;
}

bool platform_capture_screen(GlobalContext *ctx, int slot_index) {
    CGImageRef image = CGDisplayCreateImage(CGMainDisplayID());
    if (!image) {
        return false;
    }

    size_t width   = CGImageGetWidth(image);
    size_t height  = CGImageGetHeight(image);
    size_t rowBytes = CGImageGetBytesPerRow(image);

    CGDataProviderRef provider = CGImageGetDataProvider(image);
    CFDataRef        data     = CGDataProviderCopyData(provider);
    const UInt8    *buff     = CFDataGetBytePtr(data);

    uint8_t *dst = ctx->slot[slot_index].rgba;

    // CGImage возвращает BGRA в порядке байт на little-endian
    for (int y = 0; y < (int)height; y++) {
        const UInt8 *srcRow = buff + y * rowBytes;
        for (int x = 0; x < (int)width; x++) {
            UInt8 b = srcRow[x*4 + 0];
            UInt8 g = srcRow[x*4 + 1];
            UInt8 r = srcRow[x*4 + 2];
            UInt8 a = srcRow[x*4 + 3];
            size_t off = (size_t)(y * width + x) * 4;
            dst[off + 0] = r;
            dst[off + 1] = g;
            dst[off + 2] = b;
            dst[off + 3] = a;
        }
    }

    CFRelease(data);
    CGImageRelease(image);
    return true;
}
