SRCS = osp-transport.c osp.c
OBJS = $(SRCS:.c=.o)
DEPS = $(OBJS:.o=.d)
CFLAGS = -I../ -ggdb3
LDLIBS = -ldriver -pthread -lm
LDFLAGS = -L../driver 
NAME=osp

.PHONY: all lib clean

all: deps lib example

lib: lib$(NAME).a

example: example.o $(OBJS)

lib$(NAME).a: $(OBJS)
		$(AR) rcs $@ $^

deps: $(OBJS:.o=.d)

check: check.o $(OBJS)
		$(CC) $^ -o $@ $(LDFLAGS) -lcheck -lrt -lm

%.d: %.c
		$(CC) -c $(CFLAGS) -MM -MF $@ $<

clean:
		rm -rf example
		rm -rf check
		rm -rf *.o
		rm -rf *.d
		rm -rf *.a

ifneq ($(MAKECMDGOALS), clean)
-include $(OBJS:.o=.d)
endif

