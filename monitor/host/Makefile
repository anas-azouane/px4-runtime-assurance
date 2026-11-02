MAVLINK ?= ../../third_party/c_library_v2
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -D_GNU_SOURCE -Wall -Wextra -Werror -Wno-address-of-packed-member
CPPFLAGS += -I../include -I$(MAVLINK)

SRC := main_host.c $(wildcard ../src/rules/*.c) \
       $(addprefix ../src/,arbiter.c mavlink_rx.c monitor.c override_tx.c params.c state.c)

monitor_host: $(SRC) $(wildcard ../include/monitor/*.h)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SRC) -lm -o $@

clean:
	rm -f monitor_host
.PHONY: clean
