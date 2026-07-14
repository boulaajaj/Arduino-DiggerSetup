# Host test suites: characterization (#47) + safety invariants (#131) +
# pure-domain suites (one dir per extracted domain, #117 — e.g. battery/).
# One binary per test file — characterization/invariants compile the REAL
# rc_test.ino inside the stub Arduino environment (see stubs/); pure-domain
# suites compile only the extracted src/ code WITHOUT -Istubs, so an
# accidental Arduino include in domain code fails the build loudly instead
# of silently resolving to a fake.
#   make -C tests run    build + run everything (CI + pre-commit gate)
#   make -C tests        build only
# -Istubs must come first (firmware-based suites only) so <Arduino.h> etc.
# resolve to the fakes.
# Binaries land flat in build/, so file basenames must be unique across suite
# directories (invariants files carry a test_invariant_ prefix).

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -g -O0 -Wall
INCLUDES := -Istubs -Ivendor/doctest
DOMAIN_INCLUDES := -Ivendor/doctest

# Phase D (#117): extracted sketch sources are linked into every test binary —
# the firmware delegates to them, and domain suites test them directly.
# Infrastructure sources include hardware headers, so only the stub-backed
# firmware suites compile them; the pure suites take the hardware-free set
# (an accidental hardware include in domain code still fails loudly, #172).
SKETCH_SRC_CPPS    := $(shell find ../sketches/rc_test/src -name '*.cpp' 2>/dev/null)
SKETCH_SRC_HEADERS := $(shell find ../sketches/rc_test/src -name '*.h' 2>/dev/null)
SKETCH_PURE_CPPS   := $(filter-out ../sketches/rc_test/src/infrastructure/% ../sketches/rc_test/src/application/%,$(SKETCH_SRC_CPPS))

FIRMWARE := ../sketches/rc_test/rc_test.ino \
            ../sketches/rc_test/types.h \
            ../sketches/rc_test/web_page.h \
            $(SKETCH_SRC_CPPS) $(SKETCH_SRC_HEADERS)
HARNESS  := $(wildcard stubs/*.h) characterization/FirmwareUnderTest.h \
            $(wildcard invariants/*.h)

TESTS := $(wildcard characterization/test_*.cpp) \
         $(wildcard invariants/test_*.cpp) \
         $(wildcard battery/test_*.cpp) \
         $(wildcard thermal/test_*.cpp) \
         $(wildcard telemetry/test_*.cpp) \
         $(wildcard operator_input/test_*.cpp) \
         $(wildcard drive/test_*.cpp) \
         $(wildcard safety/test_*.cpp) \
         $(wildcard alerts/test_*.cpp)
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

build/%: battery/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

build/%: thermal/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

build/%: telemetry/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

build/%: operator_input/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

build/%: drive/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

build/%: safety/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

build/%: alerts/%.cpp $(SKETCH_PURE_CPPS) $(SKETCH_SRC_HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(DOMAIN_INCLUDES) $< $(SKETCH_PURE_CPPS) -o $@

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
