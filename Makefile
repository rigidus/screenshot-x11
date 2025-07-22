CC      := gcc

PNG_CFLAGS := $(shell pkg-config --cflags libpng)
PNG_LIBS   := $(shell pkg-config --libs   libpng)

CFLAGS  := -O3 -std=c17 -Wall -Wextra -pedantic -D_GNU_SOURCE $(PNG_CFLAGS)
SRCPATH := screenshot
BIN     := xcape_pipe

ARCH := $(shell uname -m)
ifeq ($(ARCH),x86_64)
    SIMD_FLAGS := -msse4.1
else
    SIMD_FLAGS :=
endif


ifeq ($(OS),Windows_NT)
    PLATFORM           := windows
    PLATFORM_SRC       := $(SRCPATH)/windows.c
    PLATFORM_LIBS      := -lgdi32 -luser32 -lkernel32 $(PNG_LIBS)
    XLIBS              :=
    EXE_SUFFIX         := .exe
    PLATFORM_DEF       := -DPLATFORM_WINDOWS $(SIMD_FLAGS)
    PLATFORM_CFLAGS    := $(CFLAGS) $(PLATFORM_DEF) -mconsole -I$(SRCPATH)
    PLATFORM_SHOT_PATH := -DSCREENSHOT_PATH=\"C:/Tmp/\"
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
		CC                 := /opt/homebrew/bin/gcc-15
        PLATFORM           := macos
        PLATFORM_SRC       := $(SRCPATH)/macos.c
        PLATFORM_LIBS      := -framework Cocoa -framework QuartzCore $(PNG_LIBS)
        XLIBS              :=
        EXE_SUFFIX         :=
        PLATFORM_DEF       := -DPLATFORM_MACOS $(SIMD_FLAGS)
        PLATFORM_CFLAGS    := $(CFLAGS) $(PLATFORM_DEF) -I$(SRCPATH)
        PLATFORM_SHOT_PATH := -DSCREENSHOT_PATH=\"./pics/\"
    else ifeq ($(UNAME_S),Linux)
        PLATFORM           := linux
        PLATFORM_SRC       := $(SRCPATH)/linux.c
        PLATFORM_LIBS      := -lX11 -lXext -lnuma -lpthread $(PNG_LIBS)
        XLIBS              := -lX11 -lXext
        EXE_SUFFIX         :=
        PLATFORM_DEF       := -DPLATFORM_LINUX $(SIMD_FLAGS)
        PLATFORM_CFLAGS    := $(CFLAGS) $(PLATFORM_DEF) -I$(SRCPATH) -pthread
        PLATFORM_SHOT_PATH := -DSCREENSHOT_PATH=\"./tmp/\"
    else
        $(error Unsupported platform: $(UNAME_S))
    endif
endif


COMMON      := xcape_pipe.c processing.c debug.c common.h
COMMON_SRCS := $(addprefix $(SRCPATH)/,$(COMMON))


$(BIN)$(EXE_SUFFIX): $(COMMON_SRCS) $(PLATFORM_SRC)
	$(CC)  \
	$(PLATFORM_CFLAGS)  \
	$(PLATFORM_SHOT_PATH)  \
	$(COMMON_SRCS)  \
	$(PLATFORM_SRC)  \
	$(PLATFORM_LIBS)  \
	-o $(BIN)$(EXE_SUFFIX)


clean:
	rm -f $(BIN)$(EXE_SUFFIX) frame_*.png *_*.png *.bmp qimg_* dbg_* tmp/*


dbg:
	./xcape_pipe --slots=1


all: $(BIN)$(EXE_SUFFIX)


.PHONY: clean dbg all
