CC     := gcc
CFLAGS := -O3 -std=c11 -Wall -Wextra -pedantic
LIBPNG := -lpng
XLIBS  := -lX11 -lXext          # нужны только для capture, можно оставить

SRC    := src
BIN    := capture invert linefinder   # ← добавили

capture:   $(SRC)/capture.c
	$(CC) $(CFLAGS) $< -o $@ $(XLIBS) $(LIBPNG)

invert:    $(SRC)/invert.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBPNG)

linefinder: $(SRC)/linefinder.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBPNG)

clean:
	rm -f $(BIN) frame_*.png *_*.png
.PHONY: all clean
