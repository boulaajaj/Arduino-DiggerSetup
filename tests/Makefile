# Host test suites: characterization (#47) + safety invariants (#131) +
# pure-domain suites (one dir per extracted domain, #117 — e.g. battery/).
# One binary per test file — characterization/invariants compile the REAL
# rc_test.ino inside the stub Arduino environment (see stubs/); domain suites
# compile only the extracted src/ code (no Arduino, no stubs needed — but
# -Istubs stays first so any transitively-included core header resolves to
# the fakes).
#   make -C tests run    build + run everything (CI + pre-commit gate)
#   make -C tests        build only
# -Istubs must come first so <Arduino.h> etc. resolve to the fakes.
# Binaries land flat in build/, so file basenames must be unique across suite
# directories (invariants files carry a test_invariant_ prefix).

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -g -O0 -Wall
INCLUDES := -Istubs -Ivendor/doctest

# Phase D (#117): extracted sketch sources are linked into every test binary —
# the firmware delegates to them, and domain suites test them directly.
SKETCH_SRC_CPPS    := $(shell find ../sketches/rc_test/src -name '*.cpp' 2>/dev/null)
SKETCH_SRC_HEADERS := $(shell find ../sketches/rc_test/src -name '*.h' 2>/dev/null)

FIRMWARE := ../sketches/rc_test/rc_test.ino \
            ../sketches/rc_test/types.h \
            ../sketches/rc_test/web_page.h \
            $(SKETCH_SRC_CPPS) $(SKETCH_SRC_HEADERS)
HARNESS  := $(wildcard stubs/*.h) characterization/FirmwareUnderTest.h \
            $(wildcard invariants/*.h)

TESTS := $(wildcard characterization/test_*.cpp) \
         $(wildcard invariants/test_*.cpp) \
         $(wildcard battery/test_*.cpp)
BINS  := $(addprefix build/,$(notdir $(TESTS:.cpp=)))

# A duplicate basename across suite directories would silently shadow one
# binary (flat build/ + same-target pattern rules) while CI stayed green —
# fail the build loudly instead. Checked across ALL suite dirs pairwise:
# $(sort) dedupes, so a count mismatch means at least one duplicate.
TEST_BASENAMES := $(notdir $(TESTS))
ifneq ($(words $(TEST_BASENAMES)),$(words $(sort $(TEST_BASENAMES))))
DUPLICATE_BASENAMES := $(strip $(foreach basename,$(sort $(TEST_BASENAMES)),\
  $(if $(filter-out 1,$(words $(filter $(basename),$(TEST_BASENAMES)))),$(basename))))
$(error duplicate test file basenames across suite directories: $(DUPLICATE_BASENAMES))
endif

all: $(BINS)

build/%: characterization/%.cpp $(HARNESS) $(FIRMWARE)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(SKETCH_SRC_CPPS) -o $@

build/%: invariants/%.cpp $(HARNESS) $(FIRMWARE)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(SKETCH_SRC_CPPS) -o $@

build/%: battery/%.cpp $(SKETCH_SRC_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(SKETCH_SRC_CPPS) -o $@

run: all
	@status=0; \
	for t in $(BINS); do \
	  echo "== $$t =="; \
	  ./$$t || status=1; \
	done; \
	exit $$status

clean:
	rm -rf build

.PHONY: all run clean
