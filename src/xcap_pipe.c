/*
 * xcap_pipe.c – first iteration (skeleton)
 * ---------------------------------------
 * Pipeline screencapture → RGB555 quant → region analysis → PNG output
 *
 * This is **just** the skeleton showing high‑level structure, thread layout,
 * data structures, and memory allocation strategy.  Functions are mostly
 * stubs with TODO markers – they will be filled in during subsequent steps.
 *
 * Build (Linux, gcc ≥ 10):
 *   gcc -O2 -march=native -pthread -lnuma -lX11 -lXext -lpng \
 *       -std=c17 -Wall -Wextra -o xcap_pipe xcap_pipe.c
 */

#define _GNU_SOURCE             /* pthread_setaffinity_np */
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <png.h>
#include <numa.h>

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
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* -------------------------------------------------------------------------- */
/*  Configuration constants (may be overridden via CLI)                        */
/* -------------------------------------------------------------------------- */

#define DEFAULT_SLOTS   4       /* ring‑buffer slots                         */
#define MAX_SLOTS       8       /* hard upper bound for simplicity           */
#define BLOCK_SIZE      32      /* pixel block analysed as a unit            */
#define MIN_LEAF        16      /* stop splitting when both sides < 16 px    */
#define COLOR_MIXED     0xFFFF  /* sentinel value for non‑uniform rectangle  */

/* -------------------------------------------------------------------------- */
/*  Simple helpers                                                             */
/* -------------------------------------------------------------------------- */

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

/* -------------------------------------------------------------------------- */
/*  Topology – immutable description of the binary split tree                  */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint16_t x0, y0, x1, y1; /* inclusive coords                            */
    uint32_t L, R;          /* indices of children or UINT32_MAX if leaf   */
} RectTopo;

/* -------------------------------------------------------------------------- */
/*  Per‑frame mutable data                                                     */
/* -------------------------------------------------------------------------- */

enum slot_state { FREE, RAW_READY, IN_PROGRESS, QUANT_DONE };

typedef struct {
    _Atomic enum slot_state st;
    _Atomic uint32_t        leaf_todo;

    uint64_t                t_start;   /* ns timestamp when capture began  */

    uint8_t  *raw;     /* pointer to ARGB8888 area within big mmap */
    uint16_t *quant;   /* pointer to RGB555 area                   */
    uint16_t *color;   /* color array [node_cnt]                   */
} FrameSlot;

/* -------------------------------------------------------------------------- */
/*  Global / shared configuration                                              */
/* -------------------------------------------------------------------------- */

typedef struct {
    int       screen_w, screen_h;
    uint32_t  node_cnt, leaf_cnt;
    uint32_t  slots;           /* ring size                                   */
    uint32_t  workers;         /* threads in worker pool                      */

    RectTopo *topo;            /* immutable tree topology                     */
    FrameSlot slots_arr[MAX_SLOTS];

    uint8_t  *bigmem;          /* mmap area start                             */
    size_t    bigmem_size;

    /* X11 */
    Display  *dpy;
    Window    root;
    XImage   *ximg;
    XShmSegmentInfo shm;
} Ctx;

static Ctx g; /* single, process‑wide */

/* -------------------------------------------------------------------------- */
/*  Forward declarations (threads & helpers)                                   */
/* -------------------------------------------------------------------------- */

static void *capture_thread(void *arg);
static void *worker_thread(void *arg);
static void *serializer_thread(void *arg);

static void build_topology(int w, int h);
static void allocate_bigmem(void);
static void init_x11(void);

/* -------------------------------------------------------------------------- */
/*  Entry point                                                                */
/* -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    /* --- parse CLI -------------------------------------------------------- */
    g.slots = DEFAULT_SLOTS;
    for (int i = 1; i < argc; ++i) {
        if (strncmp(argv[i], "--slots=", 8) == 0) {
            g.slots = (uint32_t)atoi(argv[i] + 8);
            if (g.slots < 2 || g.slots > MAX_SLOTS) {
                fprintf(stderr, "slots must be 2..%d\n", MAX_SLOTS);
                return EXIT_FAILURE;
            }
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return EXIT_FAILURE;
        }
    }

    /* --- detect CPU & NUMA ------------------------------------------------ */
    int cores = sysconf(_SC_NPROCESSORS_ONLN);
    g.workers = (cores > 4) ? (cores - 3) : 1;

    fprintf(stdout, "[init] cores=%d, workers=%u, slots=%u\n",
            cores, g.workers, g.slots);

    /* --- X11 init --------------------------------------------------------- */
    init_x11();

    /* --- build immutable tree -------------------------------------------- */
    build_topology(g.screen_w, g.screen_h);

    /* --- allocate ring buffer + aux memory -------------------------------- */
    allocate_bigmem();

    /* --- spawn threads ---------------------------------------------------- */
    pthread_t cap_tid, ser_tid;
    pthread_t *wrk_tid = calloc(g.workers, sizeof(pthread_t));

    if (pthread_create(&cap_tid, NULL, capture_thread, NULL)) die("pthread_create capture");
    if (pthread_create(&ser_tid, NULL, serializer_thread, NULL)) die("pthread_create serializer");
    for (uint32_t i = 0; i < g.workers; ++i)
        if (pthread_create(&wrk_tid[i], NULL, worker_thread, (void*)(uintptr_t)i))
            die("pthread_create worker");

    /* --- wait (infinite) -------------------------------------------------- */
    pthread_join(cap_tid, NULL);
    pthread_join(ser_tid, NULL);
    for (uint32_t i = 0; i < g.workers; ++i)
        pthread_join(wrk_tid[i], NULL);

    return 0;
}

/* -------------------------------------------------------------------------- */
/*  X11 capture initialisation (single‑threaded)                               */
/* -------------------------------------------------------------------------- */

static void init_x11(void)
{
    if (!XInitThreads())
        die("XInitThreads failed – Xlib not thread‑safe");

    g.dpy = XOpenDisplay(NULL);
    if (!g.dpy) die("XOpenDisplay");

    int screen = DefaultScreen(g.dpy);
    g.root     = RootWindow(g.dpy, screen);
    g.screen_w = DisplayWidth(g.dpy, screen);
    g.screen_h = DisplayHeight(g.dpy, screen);

    if (!XShmQueryExtension(g.dpy))
        die("XShm not available");

    g.ximg = XShmCreateImage(g.dpy, DefaultVisual(g.dpy, screen),
                             DefaultDepth(g.dpy, screen), ZPixmap,
                             NULL, &g.shm,
                             g.screen_w, g.screen_h);
    if (!g.ximg) die("XShmCreateImage");

    g.shm.shmid = shmget(IPC_PRIVATE,
                         g.ximg->bytes_per_line * g.ximg->height,
                         IPC_CREAT | 0600);
    if (g.shm.shmid < 0) die("shmget");

    g.shm.shmaddr = g.ximg->data = shmat(g.shm.shmid, NULL, 0);
    if (g.shm.shmaddr == (char*)-1) die("shmat");

    if (!XShmAttach(g.dpy, &g.shm))
        die("XShmAttach");
    XSync(g.dpy, False);

    fprintf(stdout, "[init] X11 %dx%d\n", g.screen_w, g.screen_h);
}

/* -------------------------------------------------------------------------- */
/*  Build binary split tree once                                              */
/* -------------------------------------------------------------------------- */

static void split_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                       uint32_t parent_idx)
{
    /* TODO: recursive (iterative) build into g.topo                          */
    (void)x0; (void)y0; (void)x1; (void)y1; (void)parent_idx;
}

static void build_topology(int w, int h)
{
    /* Rough upper bound on nodes (safe) */
    uint32_t max_leaf = (w * h + MIN_LEAF*MIN_LEAF - 1) / (MIN_LEAF*MIN_LEAF);
    g.node_cnt  = max_leaf * 2;        /* conservative */
    g.leaf_cnt  = max_leaf;

    g.topo = calloc(g.node_cnt, sizeof(RectTopo));
    if (!g.topo) die("calloc topo");

    /* build root */
    split_rect(0, 0, w, h, UINT32_MAX);

    fprintf(stdout, "[init] topo: nodes<=%u, leaves<=%u\n", g.node_cnt, g.leaf_cnt);
}

/* -------------------------------------------------------------------------- */
/*  Memory allocator – mmap a single region                                    */
/* -------------------------------------------------------------------------- */

static void allocate_bigmem(void)
{
    size_t raw_sz   = (size_t)g.screen_w * g.screen_h * 4;    /* ARGB */
    size_t quant_sz = (size_t)g.screen_w * g.screen_h * 2;    /* RGB555 */
    size_t color_sz = (size_t)g.node_cnt * sizeof(uint16_t);

    size_t per_slot = raw_sz + quant_sz + color_sz;
    g.bigmem_size   = per_slot * g.slots;

    g.bigmem = mmap(NULL, g.bigmem_size, PROT_READ | PROT_WRITE,
                    MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
    if (g.bigmem == MAP_FAILED) die("mmap bigmem");

    /* simple layout:  [raw][quant][color] × slots */
    for (uint32_t i = 0; i < g.slots; ++i) {
        uint8_t *base = g.bigmem + i * per_slot;
        g.slots_arr[i].raw   = base;
        g.slots_arr[i].quant = (uint16_t*)(base + raw_sz);
        g.slots_arr[i].color = (uint16_t*)(base + raw_sz + quant_sz);
        g.slots_arr[i].st    = FREE;
    }

    fprintf(stdout, "[init] bigmem %.2f MiB\n", g.bigmem_size / (1024.0*1024.0));
}

/* -------------------------------------------------------------------------- */
/*  Capture thread – alarm + frame grab                                        */
/* -------------------------------------------------------------------------- */

static void *capture_thread(void *arg)
{
    (void)arg;
    const uint64_t period_ns = 250000000ull; /* 250 ms */
    uint32_t slot_idx = 0;
    uint64_t next_t = now_ns();

    while (1) {
        /* sleep until next period */
        uint64_t t_now;
        while ((t_now = now_ns()) < next_t) {
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000000 }; /* 1 ms */
            nanosleep(&ts, NULL);
        }
        next_t += period_ns;

        FrameSlot *s = &g.slots_arr[slot_idx];
        if (atomic_load_explicit(&s->st, memory_order_acquire) != FREE) {
            fprintf(stdout, "[capture] drop: slot %u busy\n", slot_idx);
            slot_idx = (slot_idx + 1) % g.slots;
            continue;
        }

        /* grab X11 image into shared memory */
        if (!XShmGetImage(g.dpy, g.root, g.ximg, 0, 0, AllPlanes)) {
            fprintf(stderr, "XShmGetImage failed\n");
            continue;
        }

        /* copy/convert ARGB8888 → raw slot */
        memcpy(s->raw, g.ximg->data,
               (size_t)g.screen_w * g.screen_h * 4);

        s->t_start = now_ns();
        atomic_store_explicit(&s->st, RAW_READY, memory_order_release);

        slot_idx = (slot_idx + 1) % g.slots;
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/*  Worker thread – k‑quant + leaf analysis                                    */
/* -------------------------------------------------------------------------- */

static void *worker_thread(void *arg)
{
    uintptr_t wid = (uintptr_t)arg;

    /* pin to NUMA node */
    int nodes = numa_num_configured_nodes();
    if (nodes > 0) {
        int node = wid % nodes;
        numa_run_on_node(node);
        cpu_set_t mask; CPU_ZERO(&mask);
        int cores_per_node = sysconf(_SC_NPROCESSORS_ONLN) / nodes;
        int core_id = (wid % cores_per_node) + node * cores_per_node;
        CPU_SET(core_id, &mask);
        pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    }

    /* main loop */
    while (1) {
        /* TODO: pick RAW_READY slot → set IN_PROGRESS via CAS */
        /* TODO: quantize using SIMD → decrement leaf_todo     */
        /* TODO: analyse blocks 32×32 → write color[]          */
        /* TODO: timeout drop logic                            */
        sched_yield();
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/*  Serializer thread – merge and PNG write                                    */
/* -------------------------------------------------------------------------- */

static void *serializer_thread(void *arg)
{
    (void)arg;
    while (1) {
        /* TODO: scan slots for QUANT_DONE → merge colors → write PNG/file */
        sched_yield();
    }
    return NULL;
}
