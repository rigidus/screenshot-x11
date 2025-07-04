CC     := gcc
CFLAGS := -O3 -std=c11 -Wall -Wextra -pedantic
LIBPNG := -lpng
XLIBS  := -lX11 -lXext          # нужны только для capture, можно оставить

SRC    := src
BIN    := capture invert linefinder rectfinder rectfinder2 cclfinder cclfinder16 rectfinder_top15 xcape_pipe

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

xcape_pipe:
	$(CC) -O3 -march=native -std=c17 -Wall -Wextra -pthread src/xcap_pipe.c -lXext -lX11 -lnuma -lpng -o xcap_pipe


clean:
	rm -f $(BIN) frame_*.png *_*.png
.PHONY: all clean
