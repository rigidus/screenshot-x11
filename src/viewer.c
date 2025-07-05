/*
 * qimg_viewer.c  –   reconstruct PNG from .qimg produced by xcap_pipe
 *
 * usage:   ./qimg_viewer in.qimg out.png
 *
 * compile: gcc qimg_viewer.c -o qimg_viewer $(pkg-config --cflags --libs libpng)
 *
 * (с) 2025, public domain / MIT-like
 */
#include <png.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN_LEAF     16
#define COLOR_MIXED  0xFFFF
#define MAGIC_QIMG   0x51494D47u      /* 'QIMG' */

/* ───────────────────── topo (та же логика, что в xcap_pipe) ───────────── */
typedef struct { uint16_t x0,y0,x1,y1; uint32_t L,R; } RectTopo;
static RectTopo *topo;            /* глобально – проще освободить в конце  */
static uint32_t  node_cnt, leaf_cnt, topo_cap;

static inline uint32_t pow2_ge(uint32_t n)
{ return 1u << (32 - __builtin_clz(n-1)); }

static uint32_t push_rect(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
    if(node_cnt >= topo_cap){fprintf(stderr,"topo overflow\n"); exit(1);}
    uint32_t idx = node_cnt++;
    topo[idx]=(RectTopo){x0,y0,x1,y1, UINT32_MAX,UINT32_MAX};
    return idx;
}
static void split_recursive(uint32_t idx)
{
    uint16_t w=topo[idx].x1-topo[idx].x0,
        h=topo[idx].y1-topo[idx].y0;
    if(w<=MIN_LEAF && h<=MIN_LEAF){ leaf_cnt++; return; }

    uint32_t l,r;
    if(w>=h){
        uint16_t m=topo[idx].x0+w/2;
        l=push_rect(topo[idx].x0,topo[idx].y0,m,           topo[idx].y1);
        r=push_rect(m,            topo[idx].y0,topo[idx].x1,topo[idx].y1);
    }else{
        uint16_t m=topo[idx].y0+h/2;
        l=push_rect(topo[idx].x0,topo[idx].y0,topo[idx].x1,m);
        r=push_rect(topo[idx].x0,m,           topo[idx].x1,topo[idx].y1);
    }
    topo[idx].L=l; topo[idx].R=r;
    split_recursive(l); split_recursive(r);
}
static void build_topology(uint32_t W,uint32_t H)
{
    uint32_t blkX = (W+MIN_LEAF-1)/MIN_LEAF;
    uint32_t blkY = (H+MIN_LEAF-1)/MIN_LEAF;
    uint32_t leaves = pow2_ge(blkX)*pow2_ge(blkY);
    topo_cap = leaves*2-1;
    topo = malloc(topo_cap*sizeof(RectTopo));
    if(!topo){perror("malloc topo"); exit(1);}
    node_cnt = leaf_cnt = 0;
    push_rect(0,0,(uint16_t)W,(uint16_t)H);
    split_recursive(0);
}

/* ───────────────────── helpers ────────────────────────────────────────── */
static inline uint8_t expand5(uint16_t v)   /* 5-бит → 8-бит */
{ return (v<<3)|(v>>2); }

static void write_png(const char *name, uint32_t W,uint32_t H,
                      uint8_t *rgb)
{
    FILE *fp=fopen(name,"wb"); if(!fp){perror("png"); exit(1);}
    png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    png_infop   inf=png_create_info_struct(png);
    if(!png||!inf||setjmp(png_jmpbuf(png))){fclose(fp); exit(1);}
    png_init_io(png,fp);
    png_set_IHDR(png,inf,W,H,8,PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE,PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png,inf);
    for(uint32_t y=0;y<H;++y)
        png_write_row(png,rgb + y*W*3);
    png_write_end(png,NULL);
    png_destroy_write_struct(&png,&inf);
    fclose(fp);
}

/* ───────────────────── main ───────────────────────────────────────────── */
int main(int argc,char **argv)
{
    if(argc!=3){fprintf(stderr,"usage: %s in.qimg out.png\n",argv[0]);return 1;}

    /* 1. читаем файл целиком */
    FILE *fp=fopen(argv[1],"rb"); if(!fp){perror("qimg"); return 1;}
    fseek(fp,0,SEEK_END); long fsz=ftell(fp); rewind(fp);
    uint8_t *file = malloc(fsz);
    if (!file || fread(file,1,fsz,fp) != (size_t)fsz) {
        perror("fread");
        return 1;
    }
    fclose(fp);

    /* 2. заголовок */
    uint32_t *u32=(uint32_t*)file;
    if(u32[0]!=MAGIC_QIMG){fprintf(stderr,"not a QIMG\n"); return 1;}
    uint32_t W =  u32[1] & 0xFFFF;
    uint32_t H = (u32[1]>>16)&0xFFFF;
    uint32_t nodes = u32[2];
    uint16_t *colors = (uint16_t*)(u32+3);

    build_topology(W,H);
    if(nodes!=node_cnt){fprintf(stderr,"node mismatch\n"); return 1;}

    uint8_t  *rgb  = calloc(W*H*3,1);
    uint16_t *quant= calloc(W*H,sizeof(uint16_t));

    /* 3. раскладываем данные листов */
    uint8_t *cursor = (uint8_t*)(colors + nodes);
    for(uint32_t i=0;i<nodes;++i){
        if(topo[i].L!=UINT32_MAX) continue;          /* только листья */
        if(colors[i]==COLOR_MIXED){
            uint32_t w = topo[i].x1 - topo[i].x0;
            uint32_t h = topo[i].y1 - topo[i].y0;
            for(uint32_t y=0;y<h;++y){
                memcpy(quant + (topo[i].y0+y)*W + topo[i].x0,
                       cursor, w*2);
                cursor += w*2;
            }
        }else{
            /* заливка одним цветом */
            uint16_t col = colors[i];
            for(uint32_t y=topo[i].y0;y<topo[i].y1;++y){
                uint16_t *row = quant + y*W + topo[i].x0;
                uint32_t span = topo[i].x1 - topo[i].x0;      /* ширина листа */
                for(uint32_t x=0; x<span; ++x) {
                    row[x]=col;
                }
            }
            /* диагонали: ↘ белая, ↙ чёрная */
            uint32_t w = topo[i].x1 - topo[i].x0;
            uint32_t h = topo[i].y1 - topo[i].y0;
            uint32_t d = (w<h)? w:h;
            for(uint32_t i2=0;i2<d;++i2){
                quant[(topo[i].y0+i2)*W + (topo[i].x0+i2)] = 0x7FFF; /* white */
                quant[(topo[i].y0+i2)*W + (topo[i].x0+w-1-i2)] = 0x0000; /* black */
            }
        }
    }

    /* 4. преобразуем quant→RGB888 */
    for(uint32_t y=0;y<H;++y){
        for(uint32_t x=0;x<W;++x){
            uint16_t q = quant[y*W+x];
            uint8_t *p = rgb + (y*W+x)*3;
            p[0]=expand5((q>>10)&0x1F);
            p[1]=expand5((q>>5 )&0x1F);
            p[2]=expand5( q      &0x1F);
        }
    }

    /* 5. пишем PNG */
    write_png(argv[2],W,H,rgb);

    fprintf(stdout,"OK: %ux%u → %s\n",W,H,argv[2]);

    free(rgb); free(quant); free(topo); free(file);
    return 0;
}
