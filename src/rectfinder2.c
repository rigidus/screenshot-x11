/*  rectfinder2.c  ----------------------------------------------------------
 *  Улучшенный поиск одноцветных прямоугольных блоков.
 *  − работает с точным совпадением цвета (без допусков);
 *  − обнаруживает ВСЕ сплошные прямоугольники ≥ MIN_RECT_W × MIN_RECT_H;
 *  − слушает FIFO /tmp/screenshot_pipe, на входе имена PNG;
 *  − для блоков ≥ MIN_DRAW_W × MIN_DRAW_H инвертирует цвет и
 *    сохраняет <имя>_.png.
 *
 *  gcc -O3 -std=c11 rectfinder2.c -o rectfinder2 -lpng
 * ------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 200809L
#include <png.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <errno.h>

/* ---------- константы, можно менять ----------------------------------- */
#define PIPE_PATH   "/tmp/screenshot_pipe"

#define MIN_RECT_W  30
#define MIN_RECT_H  30
#define MIN_DRAW_W  50
#define MIN_DRAW_H  50

/* ---------- служебные векторы ----------------------------------------- */
typedef struct { int *d; size_t n, c; } IntVec;
#define IV_PUSH(v,val) do{ if((v).n==(v).c){(v).c=(v).c?((v).c*2):32;(v).d=realloc((v).d,(v).c*sizeof(int));} (v).d[(v).n++]=(val);}while(0)

typedef struct {
    int y, x1, x2;
    uint8_t r,g,b;
} Run;

typedef struct { Run *d; size_t n, c; } RunVec;
#define RV_PUSH(v,val) do{ if((v).n==(v).c){(v).c=(v).c?((v).c*2):256;(v).d=realloc((v).d,(v).c*sizeof(Run));} (v).d[(v).n++]=(val);}while(0)

/* ---------- union–find ------------------------------------------------- */
typedef struct{
    int *parent,*size;
    size_t n,c;
} UF;

static void uf_push(UF *u){
    if(u->n==u->c){
        u->c=u->c?u->c*2:256;
        u->parent=realloc(u->parent,u->c*sizeof(int));
        u->size  =realloc(u->size  ,u->c*sizeof(int));
    }
    u->parent[u->n]=u->n;
    u->size[u->n]=1;
    ++u->n;
}
static int uf_find(UF *u,int x){
    while(u->parent[x]!=x) x=u->parent[x]=u->parent[u->parent[x]];
    return x;
}
static void uf_union(UF *u,int a,int b){
    a=uf_find(u,a); b=uf_find(u,b);
    if(a==b) return;
    if(u->size[a]<u->size[b]){int t=a;a=b;b=t;}
    u->parent[b]=a; u->size[a]+=u->size[b];
}

/* ---------- изображение ----------------------------------------------- */
typedef struct{
    png_bytep *row;
    png_uint_32 w,h;
    int ch;   /* 4 (RGBA) */
} Image;

static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

/* --- загрузка png ---> rgba8 ------------------------------------------ */
static int load_png(const char *p, Image *im, png_structp *out_png){
    FILE *f=fopen(p,"rb"); if(!f){perror("open");return -1;}
    png_structp png=png_create_read_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   inf=png_create_info_struct(png);
    if(!png||!inf||setjmp(png_jmpbuf(png))){ fclose(f); return -1;}
    png_init_io(png,f); png_read_info(png,inf);

    im->w=png_get_image_width(png,inf);
    im->h=png_get_image_height(png,inf);
    png_byte ct=png_get_color_type(png,inf);
    png_byte bd=png_get_bit_depth(png,inf);
    if(bd==16) png_set_strip_16(png);
    if(ct==PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if(ct==PNG_COLOR_TYPE_GRAY || ct==PNG_COLOR_TYPE_GRAY_ALPHA) png_set_gray_to_rgb(png);
    if(!(ct&PNG_COLOR_MASK_ALPHA)) png_set_filler(png,0xFF,PNG_FILLER_AFTER);
    png_read_update_info(png,inf);

    im->row=png_malloc(png,im->h*sizeof(png_bytep));
    for(size_t y=0;y<im->h;++y)
        im->row[y]=png_malloc(png,png_get_rowbytes(png,inf));
    png_read_image(png,im->row);
    fclose(f);
    im->ch=4;
    *out_png=png;
    return 0;
}

/* --- сохранение png ---------------------------------------------------- */
static int save_png(const char *p,const Image *im){
    FILE *f=fopen(p,"wb"); if(!f){perror("open out");return -1;}
    png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   inf=png_create_info_struct(png);
    if(!png||!inf||setjmp(png_jmpbuf(png))){ fclose(f); return -1;}
    png_init_io(png,f);
    png_set_IHDR(png,inf,im->w,im->h,8,PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE,PNG_COMPRESSION_TYPE_DEFAULT,PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png,inf);
    png_write_image(png,(png_bytepp)im->row);
    png_write_end(png,NULL);
    fclose(f); png_destroy_write_struct(&png,&inf);
    return 0;
}

/* --- helpers ----------------------------------------------------------- */
static inline uint32_t pix32(const Image *im,int x,int y){
    png_bytep p=im->row[y]+x*im->ch;
    return (p[0]<<16)|(p[1]<<8)|p[2];
}
static inline void set_pix(Image *im,int x,int y,uint8_t r,uint8_t g,uint8_t b){
    png_bytep p=im->row[y]+x*im->ch; p[0]=r; p[1]=g; p[2]=b;
}

/* ---------- обработка одного файла ------------------------------------ */
typedef struct{
    int minX,maxX,minY,maxY,area;
    uint8_t r,g,b;
    char used;
} AccRect;

typedef struct{ int x,y,w,h; uint8_t r,g,b; } Rect;

typedef struct{ Rect *d; size_t n,c; } RectVec;
#define RECT_PUSH(v,val) do{ if((v).n==(v).c){(v).c=(v).c?((v).c*2):128;(v).d=realloc((v).d,(v).c*sizeof(Rect));} (v).d[(v).n++]=(val);}while(0)

static int cmp_area_desc(const void *a,const void *b){
    const Rect *A=a,*B=b;
    return (B->w*B->h)-(A->w*A->h);
}

static void process_image(const char *in_name){
    /* выходное имя */
    char out[256];
    const char *dot=strrchr(in_name,'.');
    if(dot){ size_t k=dot-in_name; memcpy(out,in_name,k); out[k]='_'; strcpy(out+k+1,dot);}
    else snprintf(out,sizeof out,"%s_",in_name);

    Image im={0}; png_structp pngr;
    if(load_png(in_name,&im,&pngr)<0){ fprintf(stderr,"read %s failed\n",in_name); return;}

    RunVec runs={0}; UF uf={0};
    IntVec prevRow={0}, currRow={0};

    /* --- PASS 1: строим прогоны + объединяем строки -------------------- */
    for(int y=0;y<(int)im.h;++y){
        currRow.n=0;
        int x=0;
        while(x<(int)im.w){
            uint32_t c=pix32(&im,x,y);
            int x1=x;
            do{ ++x; }while(x<(int)im.w && pix32(&im,x,y)==c);
            int x2=x-1;
            Run r={y,x1,x2,(c>>16)&0xFF,(c>>8)&0xFF,c&0xFF};
            RV_PUSH(runs,r);
            uf_push(&uf);
            IV_PUSH(currRow,(int)runs.n-1);
        }

        /* объединяем текущие и предыдущие строки */
        for(size_t ci=0; ci<currRow.n; ++ci){
            Run *cr=&runs.d[currRow.d[ci]];
            for(size_t pi=0; pi<prevRow.n; ++pi){
                Run *pr=&runs.d[prevRow.d[pi]];
                if(pr->x1>cr->x2) break;               /* дальше нет перекрытия */
                if(pr->x2<cr->x1) continue;
                if(pr->r==cr->r && pr->g==cr->g && pr->b==cr->b)
                    uf_union(&uf,(int)currRow.d[ci],(int)prevRow.d[pi]);
            }
        }
        /* swap rows */
        free(prevRow.d);
        prevRow=currRow;
        currRow.d=NULL; currRow.c=0;
    }
    free(prevRow.d);

    /* --- PASS 2: накапливаем bounding boxes --------------------------- */
    size_t N=runs.n;
    AccRect *acc = calloc(N,sizeof(AccRect));
    char *seen = calloc(N,sizeof(char));

    for(size_t i=0;i<N;++i){
        int root=uf_find(&uf,(int)i);
        Run *r=&runs.d[i];
        AccRect *a=&acc[root];
        if(!seen[root]){
            a->minX=r->x1; a->maxX=r->x2;
            a->minY=r->y ; a->maxY=r->y;
            a->r=r->r; a->g=r->g; a->b=r->b;
            seen[root]=1;
        }else{
            if(r->x1<a->minX) a->minX=r->x1;
            if(r->x2>a->maxX) a->maxX=r->x2;
            if(r->y<a->minY)  a->minY=r->y;
            if(r->y>a->maxY)  a->maxY=r->y;
        }
        a->area += r->x2 - r->x1 + 1;
    }

    /* --- PASS 3: фильтрация полных прямоугольников -------------------- */
    RectVec rects={0};
    for(size_t i=0;i<N;++i){
        if(!seen[i]) continue;
        AccRect *a=&acc[i];
        int w=a->maxX - a->minX + 1;
        int h=a->maxY - a->minY + 1;
        if(w>=MIN_RECT_W && h>=MIN_RECT_H && a->area==w*h){
            Rect rc={a->minX,a->minY,w,h,a->r,a->g,a->b};
            RECT_PUSH(rects,rc);
        }
    }
    free(acc); free(seen); free(runs.d); free(uf.parent); free(uf.size);

    /* --- сортировка и инвертирование крупных прямоугольников --------- */
    qsort(rects.d,rects.n,sizeof(Rect),cmp_area_desc);

    for(size_t i=0;i<rects.n;++i){
        if(rects.d[i].w<MIN_DRAW_W || rects.d[i].h<MIN_DRAW_H) break;
        uint8_t ir=255-rects.d[i].r, ig=255-rects.d[i].g, ib=255-rects.d[i].b;
        for(int y=rects.d[i].y; y<rects.d[i].y+rects.d[i].h; ++y)
            for(int x=rects.d[i].x; x<rects.d[i].x+rects.d[i].w; ++x)
                set_pix(&im,x,y,ir,ig,ib);
    }

    if(save_png(out,&im)==0)
        fprintf(stderr,"→ %s  (rects=%zu)\n",out,rects.n);

    /* --- cleanup ------------------------------------------------------ */
    for(size_t y=0;y<im.h;++y) png_free(pngr,im.row[y]);
    png_free(pngr,im.row);
    free(rects.d);
}

/* ---------- main ------------------------------------------------------- */
int main(void){
    if(mkfifo(PIPE_PATH,0666)==-1 && errno!=EEXIST) die("mkfifo");
    int fd=open(PIPE_PATH,O_RDONLY); if(fd<0) die("open fifo");
    FILE *fp=fdopen(fd,"r"); if(!fp) die("fdopen");

    char line[256];
    while(fgets(line,sizeof line,fp)){
        line[strcspn(line,"\n")]=0;
        if(line[0]) process_image(line);
    }
    close(fd);
    return 0;
}
