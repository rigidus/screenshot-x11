/*
 * xcap_pipe.c – iteration 6 (stall-timeout watchdog + end-to-end pipeline)
 * ----------------------------------------------------------------------------
 * Захватывает скриншоты X11 через XShm, квантизирует их до RGB555,
 * рекурсивно делит изображение на прямоугольники  ≥ 16×16 px, для каждого
 * листа определяет «униформный» цвет.  Равные соседние листы схлопываются.
 * Результат сериализуется в собственный бинарный формат .qimg и передаётся
 * именем файла во FIFO /tmp/screenshot_pipe.  Кадры, висящие в обработке
 * > 1 с, автоматически дропаются.
 *
 *
 */

/* ---------------------- компиляция ---------------------------------------
 * gcc -O3 -march=native -std=c17 -Wall -Wextra -pthread xcap_pipe.c -lXext -lX11 -lnuma -lpng -o xcap_pipe
 *
 * • `-lXext` **перед** `-lX11`, иначе XShm* остаётся неразрешённым.
 * • dev-пакеты:  libx11-dev, libxext-dev, libpng-dev, libnuma-dev.
 * ------------------------------------------------------------------------ */

#define _GNU_SOURCE
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <png.h>
#include <numa.h>
#include <emmintrin.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* ---------- константы и вспомогалки ------------------------------------ */
#define DEFAULT_SLOTS 4
#define MAX_SLOTS     8
#define MIN_LEAF      16
#define COLOR_MIXED   0xFFFF
#define PIPE_PATH     "/tmp/screenshot_pipe"
#define STALL_NS      1000000000ull              /* 1 с */
#define MAGIC_QIMG    0x51494D47u               /* 'Q''I''M''G' */

static inline uint64_t now_ns(void)
{
    struct timespec ts;  clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + ts.tv_nsec;
}
static void die(const char *m){ perror(m); exit(EXIT_FAILURE); }

/* ---------- структуры --------------------------------------------------- */
typedef struct { uint16_t x0,y0,x1,y1; uint32_t L,R; } RectTopo;
enum slot_state { FREE, RAW_READY, IN_PROGRESS, QUANT_DONE, SERIALIZING };

typedef struct {
    _Atomic enum slot_state st;
    uint64_t  t_start;
    uint8_t  *raw;
    uint16_t *quant;
    uint16_t *color;
} FrameSlot;

typedef struct {
    /* размеры экрана */
    int w,h;
    /* дерево */
    RectTopo *topo;
    uint32_t  node_cnt, leaf_cnt, topo_cap;
    /* слоты */
    uint32_t  slots, workers;
    FrameSlot slot[MAX_SLOTS];
    uint8_t  *bigmem;
    size_t    bigmem_sz;
    /* X11 */
    Display  *dpy; Window root; XImage *ximg; XShmSegmentInfo shm;
} Ctx;  static Ctx g;

/* ---------- объявления -------------------------------------------------- */
static void build_topology(int,int);
static void allocate_bigmem(void);
static void init_x11(void);
static void *capture_thread(void*);
static void *worker_thread(void*);
static void *serializer_thread(void*);
static void  write_qimg(const FrameSlot*, const char*);

/* ═════════════════════════ topo-builder ═══════════════════════════════════ */

/*
 * Добавляет узел-прямоугольник в уже заранее
 * выделенный g.topo[0 .. g.topo_cap-1].
 * При переполнении сразу завершаем программу,
 * чтобы не повредить память.
 */
static uint32_t
push_rect(uint16_t x0, uint16_t y0,
          uint16_t x1, uint16_t y1)
{
    if (g.node_cnt >= g.topo_cap)            /* страхуемся */
        die("topology overflow");            /* malloc too small */

    uint32_t idx = g.node_cnt++;             /* позиция для записи */

    g.topo[idx] = (RectTopo){
        .x0 = x0, .y0 = y0,
        .x1 = x1, .y1 = y1,
        .L  = UINT32_MAX,                    /* пока нет потомков   */
        .R  = UINT32_MAX
    };
    return idx;                              /* вернуть индекс узла */
}

/*
 * split_recursive(0) делит корень, а потом рекурсивно — каждый получившийся
 * под-прямоугольник, пока его ширина и высота не станут ≤ 16 px
 * (константа MIN_LEAF).
 * Финал: в g.topo лежит полный массив узлов, упорядоченный
 * "по месту создания", а в счётчиках — их количество.
 */
static void split_recursive(uint32_t idx)
{
    /* работаем через индекс, а не через сохранённый указатель */
    uint16_t w = g.topo[idx].x1 - g.topo[idx].x0;
    uint16_t h = g.topo[idx].y1 - g.topo[idx].y0;

    if (w <= MIN_LEAF && h <= MIN_LEAF) {  /* лист - базовый случай */
        g.leaf_cnt++;
        return;
    }

    uint32_t l, r;
    if (w >= h) {                          /* режем по X */
        uint16_t m = g.topo[idx].x0 + w / 2;
        l = push_rect(g.topo[idx].x0, g.topo[idx].y0, m,           g.topo[idx].y1);
        r = push_rect(m,              g.topo[idx].y0, g.topo[idx].x1, g.topo[idx].y1);
    } else {                               /* режем по Y */
        uint16_t m = g.topo[idx].y0 + h / 2;
        l = push_rect(g.topo[idx].x0, g.topo[idx].y0, g.topo[idx].x1, m);
        r = push_rect(g.topo[idx].x0, m,              g.topo[idx].x1, g.topo[idx].y1);
    }

    /* записываем индексы детей _после_ возможного realloc */
    g.topo[idx].L = l;
    g.topo[idx].R = r;

    split_recursive(l); // рекурсивно делим левый
    split_recursive(r); // и правый прямоугольники
}

/* ближайшая степень-2 ≥ n  (n>0) */
static inline uint32_t pow2_ge(uint32_t n)
{
    return 1u << (32 - __builtin_clz(n - 1));
}

/*
 * build_topology() — это функция, которая строит "скелет"
 * бинарного дерева Rect (topology) для текущего размера экрана.
 * Дерево понадобится всем остальным этапам конвейера: воркеры будут
 * отмечать цвета листьев, а сериализатор — сжимать одинаковые соседние узлы
 * merge_uniform_nodes() поднимается снизу вверх и схлопывает одинаковые цвета
 * write_qimg() сериализует дерево и листовые данные.
 */
static void build_topology(int w,int h)
{
    fprintf(stdout,"[init] topo: w=%d h=%d\n", w, h);

    /* Дерево полно: каждый нелист имеет ровно 2 потомка. */
    /*  Если известно L — количество листьев, то общее число узлов */
    /*  N = 2 × L – 1.  */
    /*  leaves = pow2_ge(ceil(W/16)) × pow2_ge(ceil(H/16)) */
    /* Сколько листьев? */
    /*  Алгоритм режет, пока обе стороны блока > 16 px. */
    /* Для 1854 × 1107: */
    /*  | шаг      | В/16 | следующий pow2 | Н/16 | следующий pow2 | */
    /*  | значения | 118  | 128            | 70   | 128            | */

    uint32_t blkX = (w + MIN_LEAF - 1) / MIN_LEAF;   /* ceil(w/16)  */
    uint32_t blkY = (h + MIN_LEAF - 1) / MIN_LEAF;   /* ceil(h/16)  */

    uint32_t leaves = pow2_ge(blkX) * pow2_ge(blkY);
    uint32_t nodes  = leaves * 2 - 1;

    /* g.node_cnt — сколько узлов уже записано; */
    /* g.leaf_cnt — сколько из них являются листьями (конечные прямоугольники); */
    /* g.topo_cap — текущий размер выделенного массива g.topo */

    g.node_cnt = g.leaf_cnt = 0; // обнуляем счётчики
    g.topo_cap = leaves*2 - 1;
    g.topo = malloc(g.topo_cap * sizeof(RectTopo));

    g.topo_cap = nodes;
    g.topo     = malloc(nodes * sizeof(RectTopo));
    if (!g.topo) die("malloc topo");

    push_rect(0, 0, (uint16_t)w, (uint16_t)h);

    /* split_recursive(0) делит корень, а потом рекурсивно — каждый получившийся */
    /* под-прямоугольник, пока его ширина и высота не станут ≤ 16 px */
    /* (константа MIN_LEAF). */
    split_recursive(0);

    /* Финал: в g.topo лежит полный массив узлов, упорядоченный */
    /* "по месту создания", а в счётчиках — их количество.  */
    fprintf(stdout, "[init] topo: nodes=%u leaves=%u\n",
            g.node_cnt, g.leaf_cnt); /* проверка: должны совпасть
                                        с nodes/leaves, т.е. всё влезло */
}

/* ═════════════════════════ память и X11 ═══════════════════════════════════ */
static void allocate_bigmem(void)
{
    /* если нужны строгие 64-байтовые выравнивания для AVX-инструкций, добавьте
       uintptr_t align = (uintptr_t)base & 63; if(align) base += 64 - align;
       перед расчётом raw.  */

    // Для каждого слота:
    //  base
    //  ├── raw   : [0 .. raw_sz)
    //  ├── quant : [raw_sz .. raw_sz+q_sz)
    //  └── color : [raw_sz+q_sz .. raw_sz+q_sz+col_sz)
    size_t raw_sz   = (size_t)g.w*g.h*4;                    // ARGB8888
    size_t q_sz     = (size_t)g.w*g.h*2;                    // RGB555
    size_t col_sz   = (size_t)g.node_cnt*sizeof(uint16_t);  // цвет каждого узла
    size_t per_slot = raw_sz+q_sz+col_sz;                   // один slot
    g.bigmem_sz     = per_slot*g.slots;                     // все slots

    // Один сплошной блок гарантирует, что все указатели живут в одной
    // непрерывной области и кэш-/TLB-дружественны.
    g.bigmem = mmap(NULL, g.bigmem_sz,
                    PROT_READ   | PROT_WRITE,
                    // виртуальное пространство, физические страницы
                    // "подкачаются" лениво при первом доступе.
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if ( g.bigmem == MAP_FAILED ) die("mmap bigmem");

    /* раздаём "нарезки" каждому слоту */
    /* Указатели сохраняем в структуру FrameSlot, а состояние */
    /* выставляем в FREE, чтобы поток захвата мог сразу */
    /* занимать первый свободный кадр. */
    for ( uint32_t i=0; i<g.slots; ++i ) {
        uint8_t *base = g.bigmem + i*per_slot;
        g.slot[i] = (FrameSlot) {
            .st    = FREE,                          // начальное состояние
            .raw   = base,                          // начало — сырой BGRA
            .quant = (uint16_t*)(base+raw_sz),      // следом — квант-буфер
            .color = (uint16_t*)(base+raw_sz+q_sz)  // в конце — массив цветов
        };
    }

    fprintf(stdout,"[init] bigmem %.1f MiB (per slot %.1f MiB)\n",
            g.bigmem_sz/1048576.0, per_slot/1048576.0);
}

static void init_x11(void)
{
    if ( !XInitThreads() ) die("XInitThreads");

    g.dpy=XOpenDisplay(NULL); if(!g.dpy) die("XOpenDisplay");

    int scr=DefaultScreen(g.dpy);
    g.root=RootWindow(g.dpy,scr);
    g.w=DisplayWidth(g.dpy,scr);
    g.h=DisplayHeight(g.dpy,scr);
    if(!XShmQueryExtension(g.dpy)) die("XShm");

    /* Создаём XImage в разделяемой памяти */
    g.ximg=XShmCreateImage(g.dpy,DefaultVisual(g.dpy,scr),
                           DefaultDepth(g.dpy,scr),ZPixmap,
                           NULL,&g.shm,g.w,g.h);
    if(!g.ximg) die("XShmCreateImage");
    g.shm.shmid = shmget(IPC_PRIVATE,
                         g.ximg->bytes_per_line*g.ximg->height,
                         IPC_CREAT|0600);
    if(g.shm.shmid<0) die("shmget");

    g.shm.shmaddr = g.ximg->data = shmat(g.shm.shmid,NULL,0);
    if ( g.shm.shmaddr==(char*)-1 )  die("shmat");
    if ( !XShmAttach(g.dpy,&g.shm) ) die("XShmAttach");
    XSync(g.dpy,False);

    fprintf(stdout,"[init] X11 %dx%d\n",g.w,g.h);
}


/* ═════════════════════════ SIMD RGB888→RGB555 ═════════════════════════════ */
static void quantize_rgb555(const uint8_t *src, uint16_t *dst, size_t px)
{
#ifdef __SSE2__
    size_t i=0;
    for(; i+4<=px; i+=4,src+=16,dst+=4){
        __m128i v=_mm_loadu_si128((const __m128i*)src);          /* BGRA BGRA ... */
        __m128i r=_mm_srli_epi32(v,16), g=_mm_srli_epi32(v,8), b=v;
        __m128i rgb=_mm_or_si128(_mm_slli_epi32(_mm_srli_epi32(r,3),10),
                                 _mm_or_si128(_mm_slli_epi32(_mm_srli_epi32(g,3),5),
                                              _mm_srli_epi32(b,3)));
        _mm_storel_epi64((__m128i*)dst,
                         _mm_packs_epi32(rgb,_mm_setzero_si128()));
    }
#else
    size_t i=0;
#endif
    for(; i<px; ++i,src+=4)
        *dst++ = (uint16_t)(((src[2]>>3)<<10)|((src[1]>>3)<<5)|(src[0]>>3));
}


/* ═════════════════════════ анализ листа ═══════════════════════════════════ */
static uint16_t leaf_uniform_color(const uint16_t *base,int stride,const RectTopo *r)
{
    const uint16_t c0=base[r->y0*stride + r->x0];
    __m128i vc0=_mm_set1_epi16(c0);

    for(uint16_t y=r->y0; y<r->y1; ++y){
        const uint16_t *row = base + y*stride + r->x0;
        uint16_t w=r->x1-r->x0, vcnt=w/8;
        for(uint16_t v=0; v<vcnt; ++v){
            if(_mm_movemask_epi8(
                   _mm_cmpeq_epi16(_mm_loadu_si128((const __m128i*)row),vc0))
               !=0xFFFF)
                return COLOR_MIXED;
            row+=8;
        }
        for(uint16_t x=r->x0+vcnt*8; x<r->x1; ++x)
            if(*row++!=c0) return COLOR_MIXED;
    }
    return c0;
}
static void analyse_leaves(const uint16_t *quant,uint16_t *color)
{
    int stride=g.w;
    for(uint32_t i=0;i<g.node_cnt;++i){
        const RectTopo *r=&g.topo[i];
        color[i]=(r->L==UINT32_MAX)?
            leaf_uniform_color(quant,stride,r):COLOR_MIXED;
    }
}
static void merge_uniform_nodes(uint16_t *color)
{
    for(int32_t i=(int32_t)g.node_cnt-1;i>=0;--i){
        const RectTopo *r=&g.topo[i]; if(r->L==UINT32_MAX) continue;
        uint16_t cL=color[r->L],cR=color[r->R];
        color[i]=(cL!=COLOR_MIXED && cL==cR)? cL:COLOR_MIXED;
    }
}


/* ---------- запись QIMG (исправлен magic) ------------------------------- */
static void write_qimg(const FrameSlot *s, const char *fn)
{
    FILE *fp=fopen(fn,"wb"); if(!fp){ perror("fopen qimg"); return; }

    uint32_t hdr[3] = {MAGIC_QIMG,
        (uint16_t)g.w | ((uint32_t)g.h<<16),
        g.node_cnt};
    fwrite(hdr,4,3,fp);

    fwrite(s->color,2,g.node_cnt,fp);

    for(uint32_t i=0;i<g.node_cnt;++i){
        const RectTopo *r=&g.topo[i];
        if(r->L!=UINT32_MAX) continue;
        if(s->color[i]==COLOR_MIXED){
            for(uint16_t y=r->y0;y<r->y1;++y){
                const uint16_t *row=s->quant + y*g.w + r->x0;
                fwrite(row,2,r->x1-r->x0,fp);
            }
        }
    }
    fclose(fp);
}

/* ---------- потоки ------------------------------------------------------ */
/* … capture_thread(), worker_thread() без функциональных изменений …      */
/* В конец каждой из трёх функций добавлен `return NULL;`                  */

/* ═════════════════════════ потоки ════════════════════════════════════════ */
static void *capture_thread(void *arg)
{
    (void)arg;
    const uint64_t period=250000000ull;            /* 250 мс, макс 4 FPS   */
    uint32_t idx=0; uint64_t next=now_ns();

    while(1){
        while(now_ns()<next){ struct timespec ts={0,1000000}; nanosleep(&ts,NULL);}
        next+=period;

        FrameSlot *s=&g.slot[idx];
        if(atomic_load_explicit(&s->st,memory_order_acquire)!=FREE){
            fprintf(stdout,"[drop] slot %u busy\n",idx);
            idx=(idx+1)%g.slots; continue;
        }
        if(!XShmGetImage(g.dpy,g.root,g.ximg,0,0,AllPlanes)){
            fprintf(stderr,"XShmGetImage failed\n"); continue;
        }
        memcpy(s->raw,g.ximg->data,(size_t)g.w*g.h*4);
        s->t_start=now_ns();
        atomic_store_explicit(&s->st,RAW_READY,memory_order_release);
        idx=(idx+1)%g.slots;
    }
    return NULL;
}

static void *worker_thread(void *arg)
{
    uintptr_t wid=(uintptr_t)arg;
    int nodes=numa_num_configured_nodes();
    if(nodes>0){
        int node=wid%nodes; numa_run_on_node(node);
        cpu_set_t m; CPU_ZERO(&m);
        int cpu_per= sysconf(_SC_NPROCESSORS_ONLN)/nodes;
        int cid=(wid%cpu_per)+node*cpu_per;
        CPU_SET(cid,&m);
        pthread_setaffinity_np(pthread_self(),sizeof(m),&m);
    }

    const uint32_t px=(uint32_t)g.w*g.h;
    for(;;){
        for(uint32_t i=0;i<g.slots;++i){
            FrameSlot *s=&g.slot[i];
            enum slot_state exp=RAW_READY;
            if(atomic_compare_exchange_strong(&s->st,&exp,IN_PROGRESS)){
                quantize_rgb555(s->raw,s->quant,px);
                analyse_leaves(s->quant,s->color);
                merge_uniform_nodes(s->color);
                atomic_store_explicit(&s->st,QUANT_DONE,memory_order_release);
            }
        }
        sched_yield();
    }

    return NULL;
}

static void *serializer_thread(void *arg)
{
    (void)arg;
    mkfifo(PIPE_PATH,0666);
    int pipe_fd=open(PIPE_PATH,O_WRONLY|O_NONBLOCK);

    char fn[64];
    while(1){
        uint64_t now=now_ns();
        for(uint32_t i=0;i<g.slots;++i){
            FrameSlot *s=&g.slot[i];

            enum slot_state exp=QUANT_DONE;
            if(atomic_compare_exchange_strong(&s->st,&exp,SERIALIZING)){
                struct timespec ts; clock_gettime(CLOCK_REALTIME,&ts);
                snprintf(fn,sizeof fn,"qimg_%ld_%09ld.qimg",
                         ts.tv_sec, ts.tv_nsec);
                write_qimg(s,fn);
                if(pipe_fd>=0) dprintf(pipe_fd,"%s\n",fn);
                atomic_store_explicit(&s->st,FREE,memory_order_release);
                continue;
            }
            enum slot_state cur=atomic_load_explicit(&s->st,memory_order_acquire);
            if((cur==IN_PROGRESS || cur==SERIALIZING)
               && now - s->t_start > STALL_NS){
                enum slot_state tmp=cur;
                if(atomic_compare_exchange_strong(&s->st,&tmp,FREE))
                    fprintf(stdout,"[stall] slot %u freed (state=%s)\n",
                            i, cur==IN_PROGRESS? "IN_PROGRESS":"SERIALIZING");
            }
        }
        sched_yield();
    }
    return NULL;
}


/* ═════════════════════════ main ══════════════════════════════════════════ */
int main(int argc,char **argv)
{
    /* Разбор аргументов: допускается только --slots=N */
    g.slots=DEFAULT_SLOTS;
    for(int i=1;i<argc;++i){
        if(strncmp(argv[i],"--slots=",8)==0){
            int v=atoi(argv[i]+8);
            if(v<2||v>MAX_SLOTS){ fprintf(stderr,"slots 2..%d\n",MAX_SLOTS); return 1; }
            g.slots=v;
        } else { fprintf(stderr,"unknown opt %s\n",argv[i]); return 1; }
    }

    /* Определяем число рабочих потоков */
    int cores=sysconf(_SC_NPROCESSORS_ONLN);
    g.workers=(cores>4)? cores-3 : 1;

    /* Инициализация X11 и общей топологии */
    init_x11();
    build_topology(g.w,g.h);
    allocate_bigmem();

    /* Создаём потоки: захвата, сериализатора и воркеры */
    pthread_t cap_tid, ser_tid;
    pthread_t *w_tid=calloc(g.workers,sizeof(pthread_t));
    pthread_create(&cap_tid,NULL,capture_thread,NULL);
    pthread_create(&ser_tid,NULL,serializer_thread,NULL);
    for(uint32_t i=0;i<g.workers;++i)
        pthread_create(&w_tid[i],NULL,worker_thread,(void*)(uintptr_t)i);

    /* Ждём (но на самом деле никогда не вернётся) */
    pthread_join(cap_tid,NULL);        /* никогда не возвращается          */
    pthread_join(ser_tid,NULL);
    for(uint32_t i=0;i<g.workers;++i) pthread_join(w_tid[i],NULL);
    return 0;
}
