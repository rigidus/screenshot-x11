CC     := gcc
CFLAGS := -O3 -std=c17 -Wall -Wextra -pedantic -D_GNU_SOURCE -msse4.1
LIBPNG := -lpng

# Настройка пути сохранения скриншотов (можно переопределить через make SCREENSHOT_PATH="/custom/path/")
ifdef SCREENSHOT_PATH
    SCREENSHOT_DEF := -DSCREENSHOT_PATH=\"$(SCREENSHOT_PATH)\"
else
    SCREENSHOT_DEF :=
endif

# Определение платформы
ifeq ($(OS),Windows_NT)
    PLATFORM := windows
    PLATFORM_LIBS := -lgdi32 -luser32 -lkernel32
    XLIBS :=
    EXE_SUFFIX := .exe
    PLATFORM_DEF := -DPLATFORM_WINDOWS
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        PLATFORM := linux
        PLATFORM_LIBS := -lX11 -lXext -lnuma -lpthread
        XLIBS := -lX11 -lXext
        EXE_SUFFIX :=
        PLATFORM_DEF := -DPLATFORM_LINUX
    endif
endif

SRC    := src
XCAPE_SRC := screenshot
BIN    := capture invert linefinder rectfinder rectfinder2 cclfinder cclfinder16 rectfinder_top15 xcape_pipe viewer

CXX ?= g++

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4)

capture:   $(SRC)/capture.c
	$(CC) $(CFLAGS) $< -o $@$(EXE_SUFFIX) $(XLIBS) $(LIBPNG)

invert:    $(SRC)/invert.c
	$(CC) $(CFLAGS) $< -o $@$(EXE_SUFFIX) $(LIBPNG)

linefinder: $(SRC)/linefinder.c
	$(CC) $(CFLAGS) $< -o $@$(EXE_SUFFIX) $(LIBPNG)

rectfinder: src/rectfinder.c
	$(CC) $(CFLAGS) $< -o $@$(EXE_SUFFIX) $(LIBPNG)

rectfinder2: src/rectfinder2.c
	$(CC) $(CFLAGS) $< -o $@$(EXE_SUFFIX) $(LIBPNG)

cclfinder: src/cclfinder.cpp
	$(CXX) $(CFLAGS) $(OPENCV_CFLAGS) $< -o $@$(EXE_SUFFIX) $(OPENCV_LIBS)

cclfinder16: src/cclfinder16.cpp
	$(CXX) $(CFLAGS) $(OPENCV_CFLAGS) $< -o $@$(EXE_SUFFIX) $(OPENCV_LIBS)

rectfinder_top15: src/rectfinder_top15.cpp
	$(CXX) $(CFLAGS) $(OPENCV_CFLAGS) $< -o $@$(EXE_SUFFIX) $(OPENCV_LIBS)

# Кроссплатформенная сборка xcape_pipe
xcape_pipe: $(XCAPE_SRC)/xcape_pipe.c $(XCAPE_SRC)/processing.c $(XCAPE_SRC)/debug.c $(XCAPE_SRC)/common.h
ifeq ($(PLATFORM),windows)
	$(CC) $(CFLAGS) $(PLATFORM_DEF) $(SCREENSHOT_DEF) -mconsole -I$(XCAPE_SRC) \
		$(XCAPE_SRC)/xcape_pipe.c $(XCAPE_SRC)/processing.c $(XCAPE_SRC)/debug.c $(XCAPE_SRC)/windows.c \
		$(PLATFORM_LIBS) $(LIBPNG) -o $@$(EXE_SUFFIX)
else
	$(CC) $(CFLAGS) $(PLATFORM_DEF) $(SCREENSHOT_DEF) -I$(XCAPE_SRC) -pthread \
		$(XCAPE_SRC)/xcape_pipe.c $(XCAPE_SRC)/processing.c $(XCAPE_SRC)/debug.c $(XCAPE_SRC)/linux.c \
		$(PLATFORM_LIBS) $(LIBPNG) -o $@$(EXE_SUFFIX)
endif

viewer: src/viewer.c
	gcc -O2 -std=c17 -Wall -Wextra $< -lpng -o $@$(EXE_SUFFIX)

# Примеры сборки с настройкой пути для скриншотов:
# make xcape_pipe                                    # Использует стандартные пути (C:\Tmp\ для Windows, /tmp/ для Linux)
# make xcape_pipe SCREENSHOT_PATH="/home/user/screenshots/"  # Пользовательский путь для Linux
# make xcape_pipe SCREENSHOT_PATH="C:\\Screenshots\\"        # Пользовательский путь для Windows

clean:
	rm -f $(BIN) frame_*.png *_*.png
	rm qimg_*
	rm dbg_*

dbg:
	./xcape_pipe --slots=1

.PHONY: all clean
