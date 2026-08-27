# Host-side unit tests. These build with the system compiler, not the
# IDF toolchain, and run on the development machine -- no Tanmatsu, no
# flashing. Only pure functions with no hardware dependency belong here.
#
#   make -C tests/host        build and run everything
#   make -C tests/host clean

CFLAGS  := -std=c11 -O2 -Wall -Wextra -Werror -I../../main
MAIN    := ../../main
TESTS   := test_yuv_convert

.PHONY: all clean
all: $(TESTS)
	@for t in $(TESTS); do echo "== $$t =="; ./$$t || exit 1; done

test_yuv_convert: test_yuv_convert.c $(MAIN)/yuv_convert.c $(MAIN)/yuv_convert.h
	$(CC) $(CFLAGS) -o $@ test_yuv_convert.c $(MAIN)/yuv_convert.c

clean:
	rm -f $(TESTS)
