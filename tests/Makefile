# Host test suites: characterization (#47) + safety invariants (#131). One
# binary per test file — each compiles the REAL rc_test.ino inside the stub
# Arduino environment (see stubs/).
#   make -C tests run    build + run everything (CI + pre-commit gate)
#   make -C tests        build only
# -Istubs must come first so <Arduino.h> etc. resolve to the fakes.
# Binaries land flat in build/, so file basenames must be unique across suite
# directories (invariants files carry a test_invariant_ prefix).

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -g -O0 -Wall
INCLUDES := -Istubs -Ivendor/doctest

FIRMWARE := ../sketches/rc_test/rc_test.ino \
            ../sketches/rc_test/types.h \
            ../sketches/rc_test/web_page.h
HARNESS  := $(wildcard stubs/*.h) characterization/FirmwareUnderTest.h \
            $(wildcard invariants/*.h)

TESTS := $(wildcard characterization/test_*.cpp) $(wildcard invariants/test_*.cpp)
BINS  := $(addprefix build/,$(notdir $(TESTS:.cpp=)))

# A duplicate basename across suite directories would silently shadow one
# binary (flat build/ + same-target pattern rules) while CI stayed green —
# fail the build loudly instead.
DUPLICATE_BASENAMES := $(filter $(notdir $(wildcard characterization/test_*.cpp)),\
                                $(notdir $(wildcard invariants/test_*.cpp)))
ifneq ($(DUPLICATE_BASENAMES),)
$(error duplicate test file basenames across suite directories: $(DUPLICATE_BASENAMES))
endif

all: $(BINS)

build/%: characterization/%.cpp $(HARNESS) $(FIRMWARE)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

build/%: invariants/%.cpp $(HARNESS) $(FIRMWARE)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

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
