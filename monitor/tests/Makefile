MAVLINK ?= ../../third_party/c_library_v2
GCOVR ?= gcovr
CFLAGS += -std=c11 -g -O0 -Wall -Wextra -Werror -Wno-address-of-packed-member
CPPFLAGS += -I../include -I$(MAVLINK)

SRC := test_monitor.c $(wildcard ../src/rules/*.c) \
       $(addprefix ../src/,arbiter.c mavlink_rx.c params.c state.c)

test: test_monitor
	./test_monitor

test_monitor: $(SRC) $(wildcard ../include/monitor/*.h)
	$(CC) $(CFLAGS) -fsanitize=address,undefined -fno-sanitize-recover=all $(CPPFLAGS) $(SRC) -lm -o $@

coverage:
	rm -rf cov && mkdir cov
	cd cov && $(CC) $(CFLAGS) --coverage $(CPPFLAGS:-I%=-I../%) $(addprefix ../,$(SRC)) -lm -o test_cov
	cd cov && ./test_cov
	$(GCOVR) --root .. --object-directory cov --filter '\.\./src/(rules/|arbiter)' \
	  --decisions --txt-metric line --fail-under-line 100 --fail-under-decision 100 \
	  --json-summary-pretty --json-summary cov/summary.json --print-summary --txt -

clean:
	rm -rf test_monitor cov
.PHONY: test coverage clean
