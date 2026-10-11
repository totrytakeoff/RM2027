.PHONY: all clean

CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra -Wpedantic

all: virtual_dbus

virtual_dbus: virtual_dbus.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f virtual_dbus

