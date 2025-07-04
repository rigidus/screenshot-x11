/*  rectfinder.c  ──────────────────────────────────────────────────────────
 *  Детектирует прямоугольные области одного цвета по списку горизонтальных
 *  линий, полученному из изображения, и инвертирует крупные блоки.
 *
 *  gcc -O3 -std=c11 rectfinder.c -o rectfinder -lpng
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

/* ---------- Настраиваемые пороги --------------------------------------- */
#define PIPE_PATH     "/tmp/screenshot_pipe"

#define MIN_LINE_LEN  30   /* ≥ столько пикселей — считаем линией        */
#define MIN_RECT_W    30   /* ≥ ширина прямоугольника, чтобы учесть      */
#define MIN_RECT_H    30   /* ≥ высота прямоугольника, чтобы учесть      */

#define MIN_DRAW_W    50   /* ≥ ширина блока, чтобы инвертировать        */
#define MIN_DRAW_H    50   /* ≥ высота блока, чтобы инвертировать        */

/* ---------- Структуры --------------------------------------------------- */
typedef struct {
    int  x, y;             /* начало линии                              */
    uint8_t r,g,b;         /* цвет                                      */
    int  len;              /* длина                                     */
    int  horizontal;       /* =1 горизонтальная                         */
    int  processed;        /* пометка при сборке прямоугольников        */
} LineInfo;

typedef struct {
    LineInfo *data;
    size_t cnt, cap;
} LineVec;

typedef struct {
    int *idx;              /* индексы LineInfo в LineVec */
    size_t cnt, cap;
} RowIdx;

typedef struct {
    int x,y,w,h;
    uint8_t r,g,b;
} RectInfo;

typedef struct {
    RectInfo *data;
    size_t cnt, cap;
} RectVec;

typedef struct {
    png_bytep *row;
    png_uint_32 w,h;
    int   channels;        /* всегда 4 (RGBA) */
} Image;

/* ---------- Утилиты ---------------------------------------------------- */
static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }
static void* xrealloc(void* p,size_t n){ p=realloc(p,n); if(!p) die("realloc"); return p; }

static void linevec_push(LineVec *v, LineInfo l){
    if(v->cnt==v->cap){ v->cap=v->cap? v->cap*2:256; v->data=xrealloc(v->data,v->cap*sizeof *v->data);}
    v->data[v->cnt++]=l;
}
static void rowidx_push(RowIdx *r, int idx){
    if(r->cnt==r->cap){ r->cap=r->cap? r->cap*2:32; r->idx=xrealloc(r->idx,r->cap*sizeof *r->idx);}
    r->idx[r->cnt++]=idx;
}
static void rectvec_push(RectVec *v, RectInfo r){
    if(v->cnt==v->cap){ v->cap=v->cap? v->cap*2:128; v->data=xrealloc(v->data,v->cap*sizeof *v->data);}
    v->data[v->cnt++]=r;
}

static inline uint32_t pixel32(const Image *im,int x,int y){
    png_bytep p = im->row[y]+x*im->channels;
    return (p[0]<<16)|(p[1]<<8)|p[2];      /* RRGGBB */
}
static inline void set_pixel(Image *im,int x,int y,uint8_t r,uint8_t g,uint8_t b){
    png_bytep p = im->row[y]+x*im->channels;
    p[0]=r; p[1]=g; p[2]=b;
}

/* ---------- Загрузка PNG ---------------------------------------------- */
static int load_png(const char *path, Image *im, png_structp *out_png){
    FILE *fp=fopen(path,"rb"); if(!fp){perror("open");return -1;}
    png_structp png=png_create_read_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop info=png_create_info_struct(png);
    if(!png||!info||setjmp(png_jmpbuf(png))){ fclose(fp); return -1; }
    png_init_io(png,fp);
    png_read_info(png,info);

    im->w=png_get_image_width(png,info);
    im->h=png_get_image_height(png,info);
    png_byte ct=png_get_color_type(png,info);
    png_byte bd=png_get_bit_depth(png,info);
    if(bd==16) png_set_strip_16(png);
    if(ct==PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if(ct==PNG_COLOR_TYPE_GRAY || ct==PNG_COLOR_TYPE_GRAY_ALPHA) png_set_gray_to_rgb(png);
    if(!(ct & PNG_COLOR_MASK_ALPHA)) png_set_filler(png,0xFF,PNG_FILLER_AFTER);
    png_read_update_info(png,info);
    im->channels=4;

    im->row=png_malloc(png,im->h*sizeof(png_bytep));
    for(size_t y=0;y<im->h;++y)
        im->row[y]=png_malloc(png,png_get_rowbytes(png,info));
    png_read_image(png,im->row);
    fclose(fp);
    *out_png=png;
    return 0;
}

/* ---------- Сохранение PNG -------------------------------------------- */
static int save_png(const char *path,const Image *im){
    FILE *fp=fopen(path,"wb"); if(!fp){perror("open out");return -1;}
    png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop info=png_create_info_struct(png);
    if(!png||!info||setjmp(png_jmpbuf(png))){ fclose(fp); return -1;}
    png_init_io(png,fp);
    png_set_IHDR(png,info,im->w,im->h,8,PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE,PNG_COMPRESSION_TYPE_DEFAULT,PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png,info);
    png_write_image(png,(png_bytepp)im->row);
    png_write_end(png,NULL);
    fclose(fp);
    png_destroy_write_struct(&png,&info);
    return 0;
}

/* ---------- Поиск горизонтальных линий -------------------------------- */
static void detect_lines(const Image *im, LineVec *lines, RowIdx *rows){
    const int W=im->w,H=im->h;
    for(int y=0;y<H;++y){
        int run=1, sx=0;
        uint32_t col0=pixel32(im,0,y);
        for(int x=1;x<W;++x){
            uint32_t c=pixel32(im,x,y);
            if(c==col0){ ++run; }
            else{
                if(run>=MIN_LINE_LEN){
                    LineInfo ln={sx,y,(col0>>16)&0xFF,(col0>>8)&0xFF,col0&0xFF,run,1,0};
                    linevec_push(lines,ln);
                    rowidx_push(&rows[y],lines->cnt-1);
                }
                col0=c; sx=x; run=1;
            }
        }
        if(run>=MIN_LINE_LEN){
            LineInfo ln={sx,y,(col0>>16)&0xFF,(col0>>8)&0xFF,col0&0xFF,run,1,0};
            linevec_push(lines,ln);
            rowidx_push(&rows[y],lines->cnt-1);
        }
    }
}

/* ---------- Сборка прямоугольников ------------------------------------ */
static void build_rects(const Image *im, LineVec *lines, RowIdx *rows, RectVec *rects){
    const int H=im->h;
    for(size_t i=0;i<lines->cnt;++i){
        LineInfo *base=&lines->data[i];
        if(base->processed || !base->horizontal) continue;

        int x1=base->x, x2=base->x+base->len-1;
        int y1=base->y, y2=y1;
        uint8_t r=base->r,g=base->g,b=base->b;
        base->processed=1;

        for(int y=y1+1;y<H;++y){
            RowIdx *row=&rows[y];
            LineInfo *cover=NULL;
            for(size_t k=0;k<row->cnt;++k){
                LineInfo *ln=&lines->data[row->idx[k]];
                if(ln->processed) continue;
                if(ln->r==r && ln->g==g && ln->b==b &&
                   ln->x<=x1 && ln->x+ln->len-1>=x2){
                    cover=ln; break;
                }
            }
            if(!cover) break;
            cover->processed=1;
            if(cover->x>x1) x1=cover->x;
            int ce=cover->x+cover->len-1;
            if(ce<x2) x2=ce;
            y2=y;
        }
        int w=x2-x1+1, h=y2-y1+1;
        if(w>=MIN_RECT_W && h>=MIN_RECT_H){
            RectInfo rc={x1,y1,w,h,r,g,b};
            rectvec_push(rects,rc);
        }
    }
}

static int cmp_area(const void *a,const void *b){
    const RectInfo *A=a,*B=b;
    int areaA=A->w*A->h, areaB=B->w*B->h;
    return areaB-areaA;
}

/* ---------- Инверсия крупных блоков ----------------------------------- */
static void invert_rects(Image *im,const RectVec *rects){
    for(size_t i=0;i<rects->cnt;++i){
        const RectInfo *rc=&rects->data[i];
        if(rc->w<MIN_DRAW_W || rc->h<MIN_DRAW_H) break; /* дальше меньше */
        uint8_t ir=255-rc->r, ig=255-rc->g, ib=255-rc->b;
        for(int y=rc->y;y<rc->y+rc->h;++y)
            for(int x=rc->x;x<rc->x+rc->w;++x)
                set_pixel(im,x,y,ir,ig,ib);
    }
}

/* ---------- Обработка одного файла ------------------------------------ */
static void process_image(const char *in_name){
    /* формируем имя out: вставляем '_' перед расширением */
    char out_name[256];
    const char *dot=strrchr(in_name,'.');
    if(dot){ size_t k=dot-in_name; memcpy(out_name,in_name,k); out_name[k]='_'; strcpy(out_name+k+1,dot); }
    else snprintf(out_name,sizeof out_name,"%s_",in_name);

    Image im={0};
    png_structp pngr;
    if(load_png(in_name,&im,&pngr)<0){ fprintf(stderr,"read %s failed\n",in_name); return; }

    /* массивы строк */
    RowIdx *rows=calloc(im.h,sizeof(RowIdx));
    LineVec lines={0};
    detect_lines(&im,&lines,rows);

    RectVec rects={0};
    build_rects(&im,&lines,rows,&rects);
    qsort(rects.data,rects.cnt,sizeof(RectInfo),cmp_area);

    invert_rects(&im,&rects);
    if(save_png(out_name,&im)==0)
        fprintf(stderr,"→ %s  (rects=%zu)\n",out_name,rects.cnt);

    /* очистка */
    for(size_t y=0;y<im.h;++y) png_free(pngr,im.row[y]);
    png_free(pngr,im.row);
    png_destroy_read_struct(&pngr,NULL,NULL);

    for(size_t y=0;y<im.h;++y) free(rows[y].idx);
    free(rows);
    free(lines.data);
    free(rects.data);
}

/* ---------- main ------------------------------------------------------- */
int main(void){
    if(mkfifo(PIPE_PATH,0666)==-1 && errno!=EEXIST) die("mkfifo");
    int fd=open(PIPE_PATH,O_RDONLY); if(fd<0) die("open fifo");
    FILE *fp=fdopen(fd,"r"); if(!fp) die("fdopen");

    char line[256];
    while(fgets(line,sizeof line,fp)){
        line[strcspn(line,"\n")] = '\0';
        if(line[0]) process_image(line);
    }
    close(fd);
    return 0;
}
