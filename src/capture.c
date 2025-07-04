/*  capture.c – захватывает максимум кадров за 10 с, пишет PNG
 *  и отправляет имя файла во FIFO /tmp/screenshot_pipe           */
#define _POSIX_C_SOURCE 200809L
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <png.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#define PIPE_PATH "/tmp/screenshot_pipe"

static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

/* ── запись PNG (RGB-данные) ────────────────────────────────────────────── */
static int save_png(const char *name, XImage *img)
{
    FILE *fp = fopen(name, "wb");
    if(!fp) return -1;

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   inf = png_create_info_struct(png);
    if(!png||!inf||setjmp(png_jmpbuf(png))){ fclose(fp); return -1; }

    png_init_io(png, fp);
    png_set_IHDR(png, inf, img->width, img->height, 8,
                 PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, inf);

    uint8_t *row = malloc(img->width*3);
    if(!row) die("malloc row");

    for(int y=0;y<img->height;++y){
        uint32_t *px = (uint32_t*)(img->data + y*img->bytes_per_line);
        uint8_t *p=row;
        for(int x=0;x<img->width;++x){
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            *p++ = (px[x]>>16)&0xFF;  /* R */
            *p++ = (px[x]>>8)&0xFF;   /* G */
            *p++ =  px[x]     &0xFF;  /* B */
#else
            *p++ =  px[x]&0xFF;
            *p++ = (px[x]>>8)&0xFF;
            *p++ = (px[x]>>16)&0xFF;
#endif
        }
        png_write_row(png,row);
    }
    free(row);
    png_write_end(png,NULL);
    png_destroy_write_struct(&png,&inf);
    fclose(fp);
    return 0;
}
/* ───────────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* открываем X11 */
    Display *dpy = XOpenDisplay(NULL);
    if(!dpy) die("XOpenDisplay");
    int screen = DefaultScreen(dpy);
    Window root = RootWindow(dpy,screen);
    int W = DisplayWidth(dpy,screen), H = DisplayHeight(dpy,screen);

    /* XShm init */
    if(!XShmQueryExtension(dpy)) die("XShm not available");
    XShmSegmentInfo shminfo={0};
    XImage *img = XShmCreateImage(dpy, DefaultVisual(dpy,screen),
                                  DefaultDepth(dpy,screen), ZPixmap,
                                  NULL,&shminfo,W,H);
    if(!img) die("XShmCreateImage");

    shminfo.shmid = shmget(IPC_PRIVATE,
                           img->bytes_per_line*img->height, IPC_CREAT|0600);
    if(shminfo.shmid<0) die("shmget");
    shminfo.shmaddr = img->data = shmat(shminfo.shmid,NULL,0);
    if(shminfo.shmaddr==(char*)-1) die("shmat");
    if(!XShmAttach(dpy,&shminfo)) die("XShmAttach");
    XSync(dpy,False);

    /* открываем FIFO (может ещё не быть) */
    mkfifo(PIPE_PATH,0666);
    int pipe_fd = open(PIPE_PATH,O_WRONLY|O_NONBLOCK);
    /* если второй программы ещё нет – fd==-1, будем пытаться позже */

    /* главный цикл 10 с */
    struct timespec t0,now;
    clock_gettime(CLOCK_MONOTONIC,&t0);
    int frame=0; char fname[64];

    do{
        if(!XShmGetImage(dpy,root,img,0,0,AllPlanes))
            die("XShmGetImage");

        snprintf(fname,sizeof fname,"frame_%04d.png",frame);
        if(save_png(fname,img)==0 && pipe_fd>=0){
            dprintf(pipe_fd,"%s\n",fname);          /* отправляем имя */
        }
        ++frame;

        if(pipe_fd<0)                              /* пробуем открыть позже */
            pipe_fd = open(PIPE_PATH,O_WRONLY|O_NONBLOCK);

        clock_gettime(CLOCK_MONOTONIC,&now);
    }while(now.tv_sec - t0.tv_sec < 10);

    fprintf(stderr,"Captured %d frames\n",frame);

    /* очистка */
    if(pipe_fd>=0) close(pipe_fd);
    XShmDetach(dpy,&shminfo);
    shmdt(shminfo.shmaddr);
    shmctl(shminfo.shmid,IPC_RMID,NULL);
    XDestroyImage(img);
    XCloseDisplay(dpy);
    return 0;
}
