/*  linefinder.c  ──────────────────────────────────────────────────────────
 *  Читает имена PNG из /tmp/screenshot_pipe, ищет монохромные горизонтальные
 *  и вертикальные линии, сортирует их по длине, линии ≥ MIN_DRAW_LEN
 *  инвертирует в копии изображения и сохраняет в файл <name>_.png
 *  Скомпилировать: gcc -O3 -std=c11 linefinder.c -o linefinder -lpng
 */
#define _POSIX_C_SOURCE 200809L
#include <png.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#define PIPE_PATH      "/tmp/screenshot_pipe"
#define MIN_LINE_LEN   30       /* фиксируем линию, если ≥ 30 px */
#define MIN_DRAW_LEN   50       /* выводим в файл, если ≥ 50 px */

typedef struct {
    int  x, y;                  /* координата начала (левый-верхний)      */
    uint8_t r, g, b;            /* цвет линии                             */
    int  len;                   /* длина в пикселях                       */
    int  horizontal;            /* 1 — горизонтальная, 0 — вертикальная  */
} LineInfo;

/* динамический массив LineInfo */
typedef struct {
    LineInfo *data;
    size_t    cnt, cap;
} LineVec;

static void  die(const char *m){ perror(m); exit(EXIT_FAILURE); }
static void* xrealloc(void *p, size_t n){ p=realloc(p,n); if(!p)die("realloc"); return p; }

static void vec_push(LineVec *v, LineInfo l)
{
    if(v->cnt==v->cap){
        v->cap = v->cap? v->cap*2 : 256;
        v->data = xrealloc(v->data, v->cap*sizeof(LineInfo));
    }
    v->data[v->cnt++] = l;
}

/* ---------- Загрузка PNG в память -------------------------------------- */
typedef struct {
    png_bytep *row;             /* массив указателей на строки            */
    png_uint_32 w,h;
    int channels;               /* 3 или 4                                */
} Image;

static int load_png(const char *path, Image *img, png_structp *out_png)
{
    FILE *fp = fopen(path,"rb");
    if(!fp){ perror("open"); return -1; }

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop info  = png_create_info_struct(png);
    if(!png||!info||setjmp(png_jmpbuf(png))){ fclose(fp); return -1; }

    png_init_io(png,fp);
    png_read_info(png,info);

    img->w = png_get_image_width (png,info);
    img->h = png_get_image_height(png,info);
    png_byte ct = png_get_color_type(png,info);
    png_byte bd = png_get_bit_depth(png,info);
    if(bd==16) png_set_strip_16(png);       /* к 8-битному удобнее */

    if(ct==PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);
    else if(ct==PNG_COLOR_TYPE_GRAY || ct==PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png);

    if(ct & PNG_COLOR_MASK_ALPHA)
        img->channels = 4;
    else{
        png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
        img->channels = 4;                 /* унифицируем до RGBA */
    }

    png_read_update_info(png,info);

    img->row = png_malloc(png, img->h*sizeof(png_bytep));
    for(size_t y=0;y<img->h;++y)
        img->row[y] = png_malloc(png, png_get_rowbytes(png,info));

    png_read_image(png, img->row);
    fclose(fp);
    *out_png = png;                        /* для дальнейшей очистки */
    return 0;
}

/* ---------- Сохранение PNG --------------------------------------------- */
static int save_png(const char *path, const Image *img)
{
    FILE *fp = fopen(path,"wb");
    if(!fp){ perror("open out"); return -1; }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop info  = png_create_info_struct(png);
    if(!png||!info||setjmp(png_jmpbuf(png))){ fclose(fp); return -1; }

    png_init_io(png,fp);
    png_set_IHDR(png,info,img->w,img->h,8,
                 PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png,info);
    png_write_image(png,(png_bytepp)img->row);
    png_write_end(png,NULL);
    fclose(fp);
    png_destroy_write_struct(&png,&info);
    return 0;
}

/* ---------- Вспомогательные утилиты ------------------------------------ */
static inline uint32_t pixel32(const Image *im, int x,int y)
{
    png_bytep p = im->row[y] + x*im->channels;
    return (p[0]<<16)|(p[1]<<8)|p[2];      /* RR GG BB -> 0xRRGGBB */
}
static inline void set_pixel(Image *im,int x,int y,uint8_t r,uint8_t g,uint8_t b)
{
    png_bytep p = im->row[y] + x*im->channels;
    p[0]=r; p[1]=g; p[2]=b;
}
/* ---------- Поиск линий ------------------------------------------------- */
static void find_lines(const Image *im, LineVec *v)
{
    const int W=im->w, H=im->h;

    /* горизонтальные ----------------------------------------------------- */
    for(int y=0;y<H;++y){
        int run=1, sx=0;
        uint32_t col0 = pixel32(im,0,y);

        for(int x=1;x<W;++x){
            uint32_t c = pixel32(im,x,y);
            if(c==col0) { ++run; }
            else{
                if(run>=MIN_LINE_LEN){
                    LineInfo ln={sx,y,
                        col0>>16,(col0>>8)&0xFF,col0&0xFF,
                        run,1};
                    vec_push(v,ln);
                }
                col0=c; sx=x; run=1;
            }
        }
        if(run>=MIN_LINE_LEN){
            LineInfo ln={sx,y, col0>>16,(col0>>8)&0xFF,col0&0xFF, run,1};
            vec_push(v,ln);
        }
    }

    /* вертикальные ------------------------------------------------------- */
    for(int x=0;x<W;++x){
        int run=1, sy=0;
        uint32_t col0 = pixel32(im,x,0);

        for(int y=1;y<H;++y){
            uint32_t c = pixel32(im,x,y);
            if(c==col0) { ++run; }
            else{
                if(run>=MIN_LINE_LEN){
                    LineInfo ln={x,sy,
                        col0>>16,(col0>>8)&0xFF,col0&0xFF,
                        run,0};
                    vec_push(v,ln);
                }
                col0=c; sy=y; run=1;
            }
        }
        if(run>=MIN_LINE_LEN){
            LineInfo ln={x,sy, col0>>16,(col0>>8)&0xFF,col0&0xFF, run,0};
            vec_push(v,ln);
        }
    }
}

/* компаратор для qsort (по длине, убыв) */
static int cmp_len(const void *a,const void *b)
{
    const LineInfo *A=a,*B=b;
    return B->len - A->len;
}

/* ---------- Обработка одного файла ------------------------------------- */
static void process_image(const char *in_name)
{
    /* формируем имя out: вставляем '_' перед расширением */
    char out_name[256];
    const char *dot=strrchr(in_name,'.');
    if(dot){
        size_t k = dot - in_name;
        memcpy(out_name,in_name,k);
        out_name[k]='_';
        strcpy(out_name+k+1,dot);          /* копируем ".png" */
    }else{
        snprintf(out_name,sizeof out_name,"%s_",in_name);
    }

    Image im={0};
    png_structp pngr;
    if(load_png(in_name,&im,&pngr)<0){ fprintf(stderr,"read %s failed\n",in_name); return; }

    LineVec vec={0};
    find_lines(&im,&vec);

    qsort(vec.data, vec.cnt, sizeof(LineInfo), cmp_len);

    /* рисуем линии ≥ MIN_DRAW_LEN инвертированным цветом */
    for(size_t i=0;i<vec.cnt;++i){
        if(vec.data[i].len < MIN_DRAW_LEN) break;    /* дальше только короче */
        uint8_t ir = 255 - vec.data[i].r;
        uint8_t ig = 255 - vec.data[i].g;
        uint8_t ib = 255 - vec.data[i].b;

        if(vec.data[i].horizontal){
            int y = vec.data[i].y;
            for(int dx=0; dx<vec.data[i].len; ++dx)
                set_pixel(&im, vec.data[i].x + dx, y, ir,ig,ib);
        }else{
            int x = vec.data[i].x;
            for(int dy=0; dy<vec.data[i].len; ++dy)
                set_pixel(&im, x, vec.data[i].y + dy, ir,ig,ib);
        }
    }

    if(save_png(out_name,&im)==0)
        fprintf(stderr,"→ %s  (lines=%zu)\n",out_name,vec.cnt);

    /* очистка памяти */
    for(size_t y=0;y<im.h;++y) png_free(pngr,im.row[y]);
    png_free(pngr,im.row);
    png_destroy_read_struct(&pngr,NULL,NULL);
    free(vec.data);
}

/* ---------- Точка входа ------------------------------------------------- */
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
        line[strcspn(line,"\n")] = '\0';
        if(line[0]) process_image(line);
    }
    close(fd);
    return 0;
}
