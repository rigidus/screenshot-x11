#ifdef PLATFORM_LINUX

#include "common.h"
#include <X11/Xlib.h>
#include <X11/extensions/XShm.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <unistd.h>
#include <numa.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

/* ========== Linux платформо-зависимые данные ========== */
typedef struct {
    Display            *dpy;
    Window              root;
    XImage             *ximg[MAX_SLOTS];
    XShmSegmentInfo     shm[MAX_SLOTS];
} LinuxPlatformData;

/* ========== Реализация платформо-зависимых функций ========== */

bool platform_init(GlobalContext *ctx) {
    LinuxPlatformData *pdata = malloc(sizeof(LinuxPlatformData));
    if (!pdata) return false;
    
    ctx->platform_data = pdata;
    
    if (!XInitThreads()) {
        free(pdata);
        return false;
    }

    pdata->dpy = XOpenDisplay(NULL);
    if (!pdata->dpy) {
        free(pdata);
        return false;
    }

    int scr = DefaultScreen(pdata->dpy);
    pdata->root = RootWindow(pdata->dpy, scr);
    ctx->w = DisplayWidth(pdata->dpy, scr);
    ctx->h = DisplayHeight(pdata->dpy, scr);
    
    if (!XShmQueryExtension(pdata->dpy)) {
        XCloseDisplay(pdata->dpy);
        free(pdata);
        return false;
    }

    /* Для каждого слота создаём свой XImage/SHM */
    for (uint32_t i = 0; i < ctx->slots; ++i) {
        XShmSegmentInfo *s = &pdata->shm[i];
        pdata->ximg[i] = XShmCreateImage(pdata->dpy,
                                        DefaultVisual(pdata->dpy, scr),
                                        DefaultDepth(pdata->dpy, scr),
                                        ZPixmap,
                                        NULL, s,
                                        ctx->w, ctx->h);
        if (!pdata->ximg[i]) {
            // Очистка уже созданных ресурсов
            for (uint32_t j = 0; j < i; ++j) {
                XShmDetach(pdata->dpy, &pdata->shm[j]);
                shmdt(pdata->shm[j].shmaddr);
                shmctl(pdata->shm[j].shmid, IPC_RMID, NULL);
                XDestroyImage(pdata->ximg[j]);
            }
            XCloseDisplay(pdata->dpy);
            free(pdata);
            return false;
        }

        size_t shmsz = pdata->ximg[i]->bytes_per_line * pdata->ximg[i]->height;
        s->shmid = shmget(IPC_PRIVATE, shmsz, IPC_CREAT | 0600);
        if (s->shmid < 0) {
            XDestroyImage(pdata->ximg[i]);
            XCloseDisplay(pdata->dpy);
            free(pdata);
            return false;
        }

        s->shmaddr = pdata->ximg[i]->data = shmat(s->shmid, NULL, 0);
        if (s->shmaddr == (char*)-1) {
            shmctl(s->shmid, IPC_RMID, NULL);
            XDestroyImage(pdata->ximg[i]);
            XCloseDisplay(pdata->dpy);
            free(pdata);
            return false;
        }

        if (!XShmAttach(pdata->dpy, s)) {
            shmdt(s->shmaddr);
            shmctl(s->shmid, IPC_RMID, NULL);
            XDestroyImage(pdata->ximg[i]);
            XCloseDisplay(pdata->dpy);
            free(pdata);
            return false;
        }
    }
    
    XSync(pdata->dpy, False);
    printf("[init] Linux X11 %dx%d\n", ctx->w, ctx->h);
    return true;
}

void platform_cleanup(GlobalContext *ctx) {
    if (!ctx->platform_data) return;
    
    LinuxPlatformData *pdata = (LinuxPlatformData*)ctx->platform_data;
    
    for (uint32_t i = 0; i < ctx->slots; ++i) {
        if (pdata->ximg[i]) {
            XShmDetach(pdata->dpy, &pdata->shm[i]);
            shmdt(pdata->shm[i].shmaddr);
            shmctl(pdata->shm[i].shmid, IPC_RMID, NULL);
            XDestroyImage(pdata->ximg[i]);
        }
    }
    
    if (pdata->dpy) {
        XCloseDisplay(pdata->dpy);
    }
    
    free(pdata);
    ctx->platform_data = NULL;
}

bool platform_capture_screen(GlobalContext *ctx, int slot_index) {
    LinuxPlatformData *pdata = (LinuxPlatformData*)ctx->platform_data;
    
    if (!XShmGetImage(pdata->dpy, pdata->root, pdata->ximg[slot_index], 0, 0, AllPlanes)) {
        return false;
    }
    
    // Устанавливаем указатель на raw данные (Linux уже RGBA)
    ctx->slot[slot_index].raw = (uint8_t*)pdata->ximg[slot_index]->data;
    
    return true;
}

#endif /* PLATFORM_LINUX */
