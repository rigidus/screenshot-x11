CC      := gcc
CFLAGS  := -O3 -std=c17 -Wall -Wextra -pedantic -D_GNU_SOURCE -msse4.1
LIBPNG  := -lpng
SRCPATH := screenshot
BIN     := xcape_pipe

ifeq ($(OS),Windows_NT)
    PLATFORM       := windows
	PLATFORM_SRC   := $(SRCPATH)/windows.c
    PLATFORM_LIBS  := -lgdi32 -luser32 -lkernel32 $(LIBPNG)
    XLIBS          :=
    EXE_SUFFIX     := .exe
    PLATFORM_DEF   := -DPLATFORM_WINDOWS
    PLATFORM_CFLAGS := $(CFLAGS) $(PLATFORM_DEF) -mconsole -I$(SRCPATH)
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        PLATFORM       := macos
		PLATFORM_SRC   := $(SRCPATH)/linux.c
        PLATFORM_LIBS  := -framework Cocoa -framework QuartzCore $(LIBPNG)
        XLIBS          :=
        EXE_SUFFIX     :=
        PLATFORM_DEF   := -DPLATFORM_MACOS
        PLATFORM_CFLAGS := $(CFLAGS) $(PLATFORM_DEF) -I$(SRCPATH)
    else ifeq ($(UNAME_S),Linux)
        PLATFORM       := linux
		PLATFORM_SRC   := $(SRCPATH)/linux.c
        PLATFORM_LIBS  := -lX11 -lXext -lnuma -lpthread $(LIBPNG)
        XLIBS          := -lX11 -lXext
        EXE_SUFFIX     :=
        PLATFORM_DEF   := -DPLATFORM_LINUX
        PLATFORM_CFLAGS := $(CFLAGS) $(PLATFORM_DEF) -I$(SRCPATH) -pthread
    else
        $(error Unsupported platform: $(UNAME_S))
    endif
endif


COMMON      := xcape_pipe.c processing.c debug.c common.h
COMMON_SRCS := $(addprefix $(SRCPATH)/,$(COMMON))


# теперь цель зависит от «общих» + одного платформенного
xcape_pipe: $(COMMON_SRCS) $(PLATFORM_SRC)
	$(CC) \
		$(PLATFORM_CFLAGS) \
		$(COMMON_SRCS) \
		$(PLATFORM_SRC) \
		$(PLATFORM_LIBS) \
		-o $@$(EXE_SUFFIX)

clean:
	rm -f $(BIN) frame_*.png *_*.png *.bmp
	rm qimg_*
	rm dbg_*

dbg:
	./xcape_pipe --slots=1

.PHONY: all clean
