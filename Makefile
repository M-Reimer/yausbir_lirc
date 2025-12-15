CXX ?= g++
CXXFLAGS ?= -O2 -Wall -Werror=overloaded-virtual -Wno-parentheses

DESTDIR=
PREFIX=/usr/local
BINDIR=$(PREFIX)/sbin
SYSTEMDUNITDIR=$(shell pkg-config --variable systemdsystemunitdir systemd 2>/dev/null)

LIBS = $(shell pkg-config --libs libusb-1.0)

all: yausbir_lirc

yausbir_lirc: main.cpp
	$(CXX) $(CXXFLAGS) $< $(LIBS) -o $@

install: all
	install -D -m 755 yausbir_lirc $(DESTDIR)$(BINDIR)/yausbir_lirc
ifneq ($(strip $(SYSTEMDUNITDIR)),)
	install -D -m 644 yausbir_lirc.service $(DESTDIR)$(SYSTEMDUNITDIR)/yausbir_lirc.service
	sed -i "s#/usr/sbin#$(BINDIR)#" $(DESTDIR)$(SYSTEMDUNITDIR)/yausbir_lirc.service
endif

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/yausbir_lirc
ifneq ($(strip $(SYSTEMDUNITDIR)),)
	rm -f $(DESTDIR)$(SYSTEMDUNITDIR)/yausbir_lirc.service
endif

clean:
	@rm -f yausbir_lirc
