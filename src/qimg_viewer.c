/*
 * qimg_viewer.c – utility to convert .qimg → PNG
 * ------------------------------------------------
 * • Reads .qimg produced by xcap_pipe.
 * • Rebuilds the same binary split tree (16‑pixel stop, longest‑side split).
 * • Reconstructs quantised RGB555 buffer:
 *     – If leaf color != 0xFFFF → fill rect solid color + draw two diagonals
 *     – else read raw quant pixels from stream.
 * • Converts RGB555 → RGB888 and writes PNG.
 *
 * Build:  gcc -O2 -std=c17 -Wall -lpng -o qimg_viewer qimg_viewer.c
 * Usage:  ./qimg_viewer in.qimg [out.png]
 */

#include <png.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define MIN_LEAF      16
#define COLOR_MIXED   0xFFFF

typedef struct { uint16_t x0,y0,x1,y1; uint32_t L,R; } RectTopo;

static RectTopo *topo=NULL; static uint32_t node_cnt=0, topo_cap=0;
static void die(const char *m){ perror(m); exit(EXIT_FAILURE);}

static uint32_t push_rect(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
    if(node_cnt==topo_cap){ topo_cap= topo_cap? topo_cap*2:1024;
        topo = realloc(topo, topo_cap*sizeof(RectTopo)); if(!topo) die("realloc topo"); }
    uint32_t idx=node_cnt++; topo[idx]=(RectTopo){x0,y0,x1,y1,UINT32_MAX,UINT32_MAX}; return idx;
}
static void split_recursive(uint32_t idx)
{
    RectTopo *r=&topo[idx]; uint16_t w=r->x1-r->x0, h=r->y1-r->y0;
    if(w<=MIN_LEAF && h<=MIN_LEAF) return;
    if(w>=h){ uint16_t mid=r->x0+w/2; uint32_t L=push_rect(r->x0,r->y0,mid,r->y1);
        uint32_t R=push_rect(mid,r->y0,r->x1,r->y1); r->L=L; r->R=R; }
    else    { uint16_t mid=r->y0+h/2; uint32_t L=push_rect(r->x0,r->y0,r->x1,mid);
        uint32_t R=push_rect(r->x0,mid,r->x1,r->y1); r->L=L; r->R=R; }
    split_recursive(r->L); split_recursive(r->R);
}

static void write_png(const char *fname, const uint8_t *rgb, int w, int h)
{
    FILE *fp=fopen(fname,"wb"); if(!fp) die("fopen png");
    png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);
    if(!png) die("png_create_write_struct");
    png_infop info=png_create_info_struct(png); if(!info) die("png_create_info_struct");
    if(setjmp(png_jmpbuf(png))) die("png error");
    png_init_io(png,fp);
    png_set_IHDR(png,info,w,h,8,PNG_COLOR_TYPE_RGB,PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png,info);
    for(int y=0;y<h;++y) png_write_row(png,(png_bytep)(rgb + y*w*3));
    png_write_end(png,NULL); png_destroy_write_struct(&png,&info); fclose(fp);
}

int main(int argc,char **argv)
{
    if(argc<2){ fprintf(stderr,"usage: %s in.qimg [out.png]\n",argv[0]); return 1; }
    const char *in=argv[1]; const char *out=(argc>2)? argv[2]:"out.png";
    FILE *fp=fopen(in,"rb"); if(!fp) die("open qimg");

    uint32_t hdr[3]; if(fread(hdr,4,3,fp)!=3){ fprintf(stderr,"bad header\n"); return 1; }
    if(hdr[0]!='QIMG'){ fprintf(stderr,"magic mismatch\n"); return 1; }
    uint16_t w = hdr[1]&0xFFFF, h = hdr[1]>>16; node_cnt = hdr[2];

    /* rebuild topology */
    topo=NULL; topo_cap=node_cnt; topo=malloc(topo_cap*sizeof(RectTopo)); if(!topo) die("malloc topo"); node_cnt=0;
    push_rect(0,0,w,h); split_recursive(0);
    if(node_cnt!=hdr[2]){ fprintf(stderr,"topology mismatch\n"); return 1; }

    /* read color array */
    uint16_t *color=malloc(node_cnt*sizeof(uint16_t)); if(!color) die("malloc color");
    if(fread(color,2,node_cnt,fp)!=node_cnt){ fprintf(stderr,"color read\n"); return 1; }

    /* allocate quant buf */
    uint16_t *quant=calloc(w*h,sizeof(uint16_t)); if(!quant) die("calloc quant");

    /* helper to fill rect */
    for(uint32_t i=0;i<node_cnt;++i){
        const RectTopo *r=&topo[i];
        if(r->L!=UINT32_MAX) continue; /* leaf */
        if(color[i]!=COLOR_MIXED){
            uint16_t c=color[i];
            for(uint16_t y=r->y0; y<r->y1; ++y){ uint16_t *row=quant+y*w+r->x0;
                for(uint16_t x=r->x0; x<r->x1; ++x) row[x-r->x0]=c; }
            /* draw diagonals */
            uint16_t ww=r->x1-r->x0, hh=r->y1-r->y0;
            for(uint16_t k=0;k<ww&&k<hh;++k){ quant[(r->y0+k)*w + (r->x0+k)] = 0x7FFF; /* white */
                quant[(r->y0+k)*w + (r->x1-1-k)] = 0x0000; /* black */ }
        } else {
            for(uint16_t y=r->y0; y<r->y1; ++y){ uint16_t *row=quant + y*w + r->x0;
                fread(row,2,r->x1-r->x0,fp); }
        }
    }
    fclose(fp);

    /* convert to RGB888 */
    uint8_t *rgb=malloc(w*h*3); if(!rgb) die("malloc rgb");
    for(uint32_t i=0;i<w*h;++i){ uint16_t q=quant[i]; rgb[i*3+0]=(q&0x1F)<<3; rgb[i*3+1]=((q>>5)&0x1F)<<3; rgb[i*3+2]=((q>>10)&0x1F)<<3; }

    write_png(out,rgb,w,h);
    fprintf(stdout,"written %s\n",out);
    return 0;
}
