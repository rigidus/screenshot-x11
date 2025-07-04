// screenshot-x11 : захват максимума кадров за 10 с в PNG
// однопоточный, X11 + XShm, libpng
#define _POSIX_C_SOURCE 200809L
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <png.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

static void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

/*----- PNG-сохранение ------------------------------------------------------*/
static bool write_png(const char *fname, XImage *img)
{
    FILE *fp = fopen(fname, "wb");
    if (!fp) return false;

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) { fclose(fp); return false; }

    png_infop info = png_create_info_struct(png);
    if (!info) { png_destroy_write_struct(&png, NULL); fclose(fp); return false; }

    if (setjmp(png_jmpbuf(png))) {          // libpng error
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return false;
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info,
                 img->width, img->height,
                 8,                               // bit depth
                 PNG_COLOR_TYPE_RGB,              // 3 channel
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    /* Подготавливаем указатели на строки RGB */
    const int w = img->width;
    const int h = img->height;
    uint8_t *row = malloc(w * 3);            // одна строка
    if (!row) die("malloc row");

    for (int y = 0; y < h; ++y) {
        uint32_t *src = (uint32_t *)(img->data + y * img->bytes_per_line);
        uint8_t *p = row;
        for (int x = 0; x < w; ++x) {
            uint32_t px = src[x];            // BGRA? BGRX? – чаще всего 0xAARRGGBB LE
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            uint8_t b =  px        & 0xFF;
            uint8_t g = (px >> 8)  & 0xFF;
            uint8_t r = (px >> 16) & 0xFF;
#else   // big-endian
            uint8_t r =  px        & 0xFF;
            uint8_t g = (px >> 8)  & 0xFF;
            uint8_t b = (px >> 16) & 0xFF;
#endif
            *p++ = r; *p++ = g; *p++ = b;
        }
        png_write_row(png, row);
    }
    free(row);

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return true;
}

/*----- Главная -------------------------------------------------------------*/
int main(void)
{
    Display *dpy = XOpenDisplay(NULL);
    if (!dpy) die("XOpenDisplay");

    int screen = DefaultScreen(dpy);
    Window root = RootWindow(dpy, screen);
    int width  = DisplayWidth(dpy, screen);
    int height = DisplayHeight(dpy, screen);

    /* Проверяем XShm */
    int maj, min;
    Bool pixmaps;
    if (!XShmQueryVersion(dpy, &maj, &min, &pixmaps))
        die("XShm not available");

    /* Создаём XImage в общей памяти */
    XShmSegmentInfo shminfo = {0};
    XImage *img = XShmCreateImage(dpy,
                                  DefaultVisual(dpy, screen),
                                  DefaultDepth(dpy, screen),
                                  ZPixmap, NULL, &shminfo,
                                  width, height);
    if (!img) die("XShmCreateImage");

    shminfo.shmid = shmget(IPC_PRIVATE,
                           img->bytes_per_line * img->height,
                           IPC_CREAT | 0600);
    if (shminfo.shmid < 0) die("shmget");

    shminfo.shmaddr = img->data = shmat(shminfo.shmid, NULL, 0);
    if (shminfo.shmaddr == (char *)-1) die("shmat");

    shminfo.readOnly = False;
    if (!XShmAttach(dpy, &shminfo)) die("XShmAttach");

    XSync(dpy, False);    // убедиться, что attach завершён

    /* Основной цикл – ровно 10 с */
    struct timespec t0, now;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    int frame = 0;
    char fname[64];

    do {
        if (!XShmGetImage(dpy, root, img, 0, 0, AllPlanes)) {
            fprintf(stderr, "XShmGetImage failed on frame %d\n", frame);
            break;
        }

        snprintf(fname, sizeof(fname), "frame_%04d.png", frame);
        if (!write_png(fname, img))
            fprintf(stderr, "PNG write failed: %s\n", fname);

        ++frame;
        clock_gettime(CLOCK_MONOTONIC, &now);

    } while ((now.tv_sec - t0.tv_sec) < 10);

    printf("Captured %d frames in 10 seconds\n", frame);

    /* Очистка */
    XShmDetach(dpy, &shminfo);
    shmdt(shminfo.shmaddr);
    shmctl(shminfo.shmid, IPC_RMID, NULL);
    XDestroyImage(img);
    XCloseDisplay(dpy);

    return 0;
}
