# Host-side build and verification. Nothing here needs an STM32 attached.
#
#   make            everything below that runs without extra tooling
#   make test       the seven test programs
#   make sim        what actuation is worth, against a Webster-timed baseline
#   make check      delay regression guard, against recorded baselines
#   make timeline   one intersection, every change of indication
#   make faults     each fault class, injected, and the monitor catching it
#   make trace      per-tick CSV into build/
#   make node       type-check the board layer against a stubbed HAL
#
# Needing tooling this repository does not vendor:
#
#   make coverage   gcov line coverage over firmware/common/app
#   make fuzz       libFuzzer over the frame parser          (needs clang)
#   make prove      CBMC over the conflict invariant         (needs cbmc)
#   make arm        cross-compile and report the footprint   (needs arm-none-eabi-gcc)
#
# Each of those four prints what is missing rather than failing obscurely.
#
# NOTE ON "make": on Windows with MinGW the binary is mingw32-make, and there
# is no "make" on PATH. Every "make X" in this repository means
# "mingw32-make X" there. CI runs GNU make on Linux, which is the
# configuration the instructions are written for.

ifeq ($(origin CC),default)
CC := gcc
endif

ifeq ($(OS),Windows_NT)
EXE := .exe
else
EXE :=
endif

BUILD := build/host
COV   := build/cov
APP   := firmware/common/app
NODE  := firmware/semaphore_node/Core
SIM   := sim
STUB  := tools/hal_stub

# The full set, not just -Wall -Wextra. Most of these cost nothing on code
# that was written with the conversions made explicit in the first place, and
# the two that did find something - a Win32 socket handle narrowed to int -
# were worth the afternoon.
WARN := -Wall -Wextra -Werror \
        -Wconversion -Wsign-conversion -Wshadow -Wpedantic \
        -Wstrict-prototypes -Wmissing-prototypes -Wcast-qual \
        -Wundef -Wdouble-promotion

CFLAGS := -std=c99 $(WARN)

# make test SAN=1
#
# Address and undefined-behaviour sanitizers over the whole suite. Not the
# default because MinGW on Windows ships no libasan, and a default that fails
# on the machine the project is developed on is a default nobody runs. CI runs
# it on Linux with clang, which is where the interesting answers are.
#
# -fno-sanitize-recover makes UBSan fail the run rather than print and carry
# on, so a CI job that is green is a job that found nothing.
ifdef SAN
CFLAGS += -g -O1 -fsanitize=address,undefined -fno-sanitize-recover=undefined
endif

APP_SRC := $(APP)/lamps.c $(APP)/intersection.c $(APP)/conflict_monitor.c \
           $(APP)/sem_proto.c $(APP)/semaphore.c $(APP)/plans.c
APP_HDR := $(wildcard $(APP)/*.h)
SIM_SRC := $(SIM)/city.c $(SIM)/udp_link.c
SIM_HDR := $(wildcard $(SIM)/*.h)
NODE_SRC := $(NODE)/Src/lamp_gpio.c $(NODE)/Src/esp_link.c \
            $(NODE)/Src/semaphore_node.c
NODE_HDR := $(wildcard $(NODE)/Inc/*.h)

INC      := -I$(APP) -I$(SIM) -Itests
NODE_INC := -I$(STUB) -I$(APP) -I$(NODE)/Inc -Itests

ifeq ($(OS),Windows_NT)
SIM_LIBS := -lws2_32
else
SIM_LIBS :=
endif

# Tests that link only the portable core.
CORE_TESTS := test_plan test_phases test_monitor test_proto test_integration \
              fuzz_proto cbmc_harness
# Tests that also link the board layer and the fake HAL.
NODE_TESTS := test_node

TESTS := $(CORE_TESTS) $(NODE_TESTS)

.PHONY: all test sim check timeline faults trace node coverage fuzz prove arm \
        clean $(TESTS)

all: test sim check node
	@echo "all checks passed"

test: $(TESTS)

## The configuration check, against plans that are wrong on purpose.
test_plan: $(BUILD)/test_plan$(EXE)
	$(BUILD)/test_plan$(EXE)

## The sequence and the length of every interval in it.
test_phases: $(BUILD)/test_phases$(EXE)
	$(BUILD)/test_phases$(EXE)

## The conflict monitor, against field output no controller would produce.
test_monitor: $(BUILD)/test_monitor$(EXE)
	$(BUILD)/test_monitor$(EXE)

## The broadcast frame, and whether its countdown is ever early.
test_proto: $(BUILD)/test_proto$(EXE)
	$(BUILD)/test_proto$(EXE)

## Forty simulated minutes, every invariant checked on every tick.
test_integration: $(BUILD)/test_integration$(EXE)
	$(BUILD)/test_integration$(EXE)

## The board layer, on a fake HAL: debounce, tick order, the fault path.
test_node: $(BUILD)/test_node$(EXE)
	$(BUILD)/test_node$(EXE)

## The frame parser against its corpus and a deterministic mutation sweep.
## This is the no-clang build of the libFuzzer target; see "make fuzz".
fuzz_proto: $(BUILD)/fuzz_proto$(EXE)
	$(BUILD)/fuzz_proto$(EXE)

## Not a proof - a check that the CBMC harnesses still compile and still
## match the code they reason about. "make prove" is the proof.
cbmc_harness: $(BUILD)/cbmc_harness$(EXE)
	$(BUILD)/cbmc_harness$(EXE)

## Mean delay per car: naive fixed, Webster-timed fixed, and actuated.
sim: $(BUILD)/sem_sim$(EXE)
	$(BUILD)/sem_sim$(EXE)

## Fails if the delay figures have moved more than the recorded tolerance.
check: $(BUILD)/sem_sim$(EXE)
	$(BUILD)/sem_sim$(EXE) --check

## Every change of indication for 60 s, with the frames that went with it.
timeline: $(BUILD)/sem_sim$(EXE)
	$(BUILD)/sem_sim$(EXE) --timeline

## Each way the field output can go wrong, and what the monitor does about it.
faults: $(BUILD)/sem_sim$(EXE)
	@for f in conflict dark shortyellow noclearance stall; do \
		$(BUILD)/sem_sim$(EXE) --fault=$$f; echo; \
	done

## Per-tick state, countdowns and queue lengths, for plotting.
trace: $(BUILD)/sem_sim$(EXE)
	$(BUILD)/sem_sim$(EXE) --csv --seconds=120 > $(BUILD)/trace_cross4.csv
	$(BUILD)/sem_sim$(EXE) --csv --seconds=120 --plan=cross4-fixed \
		> $(BUILD)/trace_fixed.csv
	@echo "wrote $(BUILD)/trace_cross4.csv and $(BUILD)/trace_fixed.csv"

# ---- build rules --------------------------------------------------------

$(BUILD)/test_%$(EXE): tests/test_%.c tests/test_util.h tests/rig.h \
                       $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_$*.c $(APP_SRC) -o $@

$(BUILD)/fuzz_proto$(EXE): tests/fuzz_proto.c $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/fuzz_proto.c $(APP_SRC) -o $@

$(BUILD)/cbmc_harness$(EXE): tests/cbmc_harness.c $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/cbmc_harness.c $(APP_SRC) -o $@

# test_node is the one test that reaches below the portable core, so it gets
# the stubbed HAL headers, the fake HAL that implements them, and the board
# layer itself.
$(BUILD)/test_node$(EXE): tests/test_node.c tests/fake_hal.c tests/fake_hal.h \
                          tests/test_util.h $(NODE_SRC) $(NODE_HDR) \
                          $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(NODE_INC) tests/test_node.c tests/fake_hal.c \
	      $(NODE_SRC) $(APP_SRC) -o $@

$(BUILD)/sem_sim$(EXE): $(SIM)/sem_sim.c $(SIM_SRC) $(SIM_HDR) \
                        $(APP_SRC) $(APP_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(SIM)/sem_sim.c $(SIM_SRC) $(APP_SRC) \
	      $(SIM_LIBS) -o $@

## The board layer never runs on the host in the shipped configuration, so it
## would never be compiled here either - and code nobody compiles is code with
## a typo in it. This builds it against a stubbed HAL that defines nothing.
## It proves the board layer compiles. tests/test_node.c is what runs it.
node: $(NODE_SRC) $(NODE_HDR) $(APP_HDR)
	@mkdir -p $(BUILD)
	@for f in $(NODE_SRC); do \
		echo "$(CC) $(CFLAGS) -c $$f"; \
		$(CC) $(CFLAGS) -I$(STUB) -I$(APP) -I$(NODE)/Inc -c $$f \
		      -o $(BUILD)/`basename $$f .c`.o || exit 1; \
	done
	@echo "board layer compiles"

# ---- things that need tooling we do not vendor --------------------------

## Line coverage over the portable core.
##
## The core is compiled once, with coverage, and every test links against
## those same objects - so the counters accumulate across the whole suite
## rather than reporting whatever the last test happened to reach.
##
## plans.c does not appear: it is a table of static data with no executable
## lines in it, which gcov correctly declines to have an opinion about.
##
## The test drivers are compiled and linked in two steps rather than one.
## gcc names the .gcno after the object it is producing, and a compile-and-
## link has no object to name it after, so it writes into the working
## directory instead - six pairs of .gcno/.gcda dropped in the project root
## every time anybody asked for a coverage number.
COV_TESTS := test_plan test_phases test_monitor test_proto test_integration \
             fuzz_proto
COV_OBJ   := $(addprefix $(COV)/,$(notdir $(APP_SRC:.c=.o)))

coverage:
	@command -v gcov >/dev/null 2>&1 || \
		{ echo "gcov not found - it ships with gcc"; exit 1; }
	@rm -rf $(COV) && mkdir -p $(COV)
	@for f in $(APP_SRC); do \
		$(CC) -std=c99 -O0 --coverage -I$(APP) -c $$f \
		      -o $(COV)/`basename $$f .c`.o || exit 1; \
	done
	@for t in $(COV_TESTS); do \
		$(CC) -std=c99 -O0 --coverage -I$(APP) -I$(SIM) -Itests \
		      -c tests/$$t.c -o $(COV)/$$t.o || exit 1; \
		$(CC) -std=c99 -O0 --coverage \
		      $(COV)/$$t.o $(COV_OBJ) -o $(COV)/$$t$(EXE) || exit 1; \
	done
	@for t in $(COV_TESTS); do $(COV)/$$t$(EXE) > /dev/null || exit 1; done
	@gcov -n -o $(COV) $(APP_SRC) > $(COV)/gcov.txt 2>/dev/null || true
	@echo
	@echo "line coverage, firmware/common/app:"
	@awk -f tools/cov_report.awk $(COV)/gcov.txt
	@echo
	@echo "(uncovered lines are where a reviewer should look first)"

## Coverage-guided fuzzing of the only parser of untrusted input.
## The same file builds without clang as a replay driver - see "make test".
fuzz:
	@command -v clang >/dev/null 2>&1 || \
		{ echo "clang not found; 'make test' runs the replay driver instead"; \
		  exit 1; }
	@mkdir -p $(BUILD)/fuzz-corpus
	clang -std=c99 -g -O1 -DFUZZING_BUILD \
	      -fsanitize=fuzzer,address,undefined \
	      -fno-sanitize-recover=undefined \
	      -I$(APP) tests/fuzz_proto.c $(APP)/sem_proto.c $(APP)/lamps.c \
	      -o $(BUILD)/fuzz_proto_libfuzzer
	$(BUILD)/fuzz_proto_libfuzzer $(BUILD)/fuzz-corpus \
	      -max_total_time=$(or $(FUZZ_SECONDS),60) -print_final_stats=1

## Bounded model checking of the property the whole project exists for.
## Slow, and the depth is the honest limit - see the header of the harness.
prove:
	@command -v cbmc >/dev/null 2>&1 || \
		{ echo "cbmc not found - https://github.com/diffblue/cbmc/releases"; \
		  exit 1; }
	@for h in harness_no_conflicting_green \
	          harness_aspect_sequence \
	          harness_plan_check_rejects_conflicting_stages; do \
		echo "=== $$h"; \
		cbmc tests/cbmc_harness.c $(APP)/intersection.c $(APP)/lamps.c \
		     $(APP)/plans.c -I$(APP) \
		     --unwind $(or $(UNWIND),41) --unwinding-assertions \
		     --function $$h || exit 1; \
	done
	@echo "all harnesses proved at depth $(or $(UNWIND),41)"

## Flash and RAM footprint of the code this project actually wrote. Excludes
## the CubeMX HAL and the startup file, which are boilerplate and would swamp
## the number that says anything about this repository.
ARM_CC ?= arm-none-eabi-gcc
ARM_SIZE ?= arm-none-eabi-size
ARM_FLAGS := -mcpu=cortex-m3 -mthumb -Os -ffunction-sections -fdata-sections

arm:
	@command -v $(ARM_CC) >/dev/null 2>&1 || \
		{ echo "$(ARM_CC) not found - install the arm-none-eabi toolchain"; \
		  exit 1; }
	@mkdir -p $(BUILD)/arm
	@for f in $(APP_SRC) $(NODE_SRC); do \
		$(ARM_CC) -std=c99 $(ARM_FLAGS) $(WARN) \
		          -I$(STUB) -I$(APP) -I$(NODE)/Inc -c $$f \
		          -o $(BUILD)/arm/`basename $$f .c`.o || exit 1; \
	done
	@echo
	@$(ARM_SIZE) -t $(BUILD)/arm/*.o
	@echo
	@echo "text = flash, data+bss = RAM. HAL and startup excluded."

clean:
	rm -rf build
