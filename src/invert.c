/*  invert.c – читает имена PNG из FIFO /tmp/screenshot_pipe,
 *  инвертирует цвета и пишет файл с подчёркиванием в имени           */
#define _POSIX_C_SOURCE 200809L
#include <png.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define PIPE_PATH "/tmp/screenshot_pipe"

static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

/* ── инвертировать и сохранить ─────────────────────────────────────────── */
static int invert_png(const char *in_name)
{
    /* формируем out_name: вставляем '_' перед расширением */
    char out_name[128];
    const char *dot = strrchr(in_name,'.');
    if(dot){
        size_t k = dot - in_name;
        memcpy(out_name,in_name,k);
        out_name[k]='_';
        strcpy(out_name+k+1,dot);          /* копируем ".png" */
    }else{
        snprintf(out_name,sizeof out_name,"%s_",in_name);
    }

    /* читаем входной PNG */
    FILE *fi = fopen(in_name,"rb");
    if(!fi){ perror("open in"); return -1; }

    png_structp pngr = png_create_read_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   infor= png_create_info_struct(pngr);
    if(!pngr||!infor||setjmp(png_jmpbuf(pngr))){ fclose(fi); return -1; }
    png_init_io(pngr,fi);
    png_read_info(pngr,infor);

    png_uint_32 w,h; int bit,ctype;
    png_get_IHDR(pngr,infor,&w,&h,&bit,&ctype,NULL,NULL,NULL);
    if(bit!=8 || (ctype!=PNG_COLOR_TYPE_RGB && ctype!=PNG_COLOR_TYPE_RGBA)){
        fprintf(stderr,"Unsupported PNG format %s\n",in_name);
        fclose(fi); return -1;
    }

    size_t channels = png_get_channels(pngr,infor);
    png_bytep *rows = png_malloc(pngr,h*sizeof(png_bytep));
    for(size_t y=0;y<h;++y)
        rows[y] = png_malloc(pngr,png_get_rowbytes(pngr,infor));
    png_read_image(pngr,rows);
    fclose(fi);
    png_destroy_read_struct(&pngr,&infor,NULL);

    /* инвертируем */
    for(size_t y=0;y<h;++y){
        png_bytep p = rows[y];
        for(size_t x=0;x<w;++x){
            p[0] = 255 - p[0];             /* R */
            p[1] = 255 - p[1];             /* G */
            p[2] = 255 - p[2];             /* B */
            p += channels;
        }
    }

    /* пишем выходной PNG */
    FILE *fo = fopen(out_name,"wb");
    if(!fo){ perror("open out"); return -1; }

    png_structp pngw = png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   infow= png_create_info_struct(pngw);
    if(!pngw||!infow||setjmp(png_jmpbuf(pngw))){ fclose(fo); return -1; }
    png_init_io(pngw,fo);
    png_set_IHDR(pngw,infow,w,h,8,ctype,PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,PNG_FILTER_TYPE_DEFAULT);
    png_write_info(pngw,infow);
    png_write_image(pngw,rows);
    png_write_end(pngw,NULL);
    fclose(fo);
    png_destroy_write_struct(&pngw,&infow);

    for(size_t y=0;y<h;++y) png_free(pngr,rows[y]);
    png_free(pngr,rows);

    fprintf(stderr,"→ %s\n",out_name);
    return 0;
}
/* ───────────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* создаём FIFO, если нет */
    if(mkfifo(PIPE_PATH,0666)==-1 && errno!=EEXIST)
        die("mkfifo");

    int fd = open(PIPE_PATH,O_RDONLY);
    if(fd<0) die("open fifo");
    FILE *fp = fdopen(fd,"r");
    if(!fp) die("fdopen");

    char line[256];
    while(fgets(line,sizeof line,fp)){
        /* убираем перевод строки */
        line[strcspn(line,"\n")] = '\0';
        if(line[0]) invert_png(line);
    }
    close(fd);
    return 0;
}
