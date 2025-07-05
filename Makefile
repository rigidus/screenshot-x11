CC     := gcc
CFLAGS := -O3 -std=c11 -Wall -Wextra -pedantic
LIBPNG := -lpng
XLIBS  := -lX11 -lXext          # нужны только для capture, можно оставить

SRC    := src
BIN    := capture invert linefinder rectfinder rectfinder2 cclfinder cclfinder16 rectfinder_top15 xcape_pipe viewer

CXX ?= g++

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4)

capture:   $(SRC)/capture.c
	$(CC) $(CFLAGS) $< -o $@ $(XLIBS) $(LIBPNG)

invert:    $(SRC)/invert.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBPNG)

linefinder: $(SRC)/linefinder.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBPNG)

rectfinder: src/rectfinder.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBPNG)

rectfinder2: src/rectfinder2.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBPNG)

cclfinder: src/cclfinder.cpp
	$(CXX) $(CFLAGS) $(OPENCV_CFLAGS) $< -o $@ $(OPENCV_LIBS)

cclfinder16: src/cclfinder16.cpp
	$(CXX) $(CFLAGS) $(OPENCV_CFLAGS) $< -o $@ $(OPENCV_LIBS)

rectfinder_top15: src/rectfinder_top15.cpp
	$(CXX) $(CFLAGS) $(OPENCV_CFLAGS) $< -o $@ $(OPENCV_LIBS)

xcape_pipe: src/xcape_pipe.c
	$(CC) -O3 -march=native -std=c17 -Wall -Wextra -pthread $<  -lXext -lX11 -lnuma -lpng -o $@

viewer: src/viewer.c
	gcc -O2 -std=c17 -Wall -Wextra $< -lpng -o $@

clean:
	rm -f $(BIN) frame_*.png *_*.png
	rm qimg_*
	rm dbg_*

dbg:
	XCAP_DEBUG=1 XCAP_DEBUG_RAW=1 XCAP_DEBUG_QUANT=1 ./xcape_pipe --slots=1

.PHONY: all clean
