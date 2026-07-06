# Host characterization suite (#47). One binary per test file — each compiles
# the REAL rc_test.ino inside the stub Arduino environment (see stubs/).
#   make -C tests run    build + run everything (CI + pre-commit gate)
#   make -C tests        build only
# -Istubs must come first so <Arduino.h> etc. resolve to the fakes.

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -g -O0 -Wall
INCLUDES := -Istubs -Ivendor/doctest

FIRMWARE := ../sketches/rc_test/rc_test.ino \
            ../sketches/rc_test/types.h \
            ../sketches/rc_test/web_page.h
HARNESS  := $(wildcard stubs/*.h) characterization/FirmwareUnderTest.h

TESTS := $(wildcard characterization/test_*.cpp)
BINS  := $(patsubst characterization/%.cpp,build/%,$(TESTS))

all: $(BINS)

build/%: characterization/%.cpp $(HARNESS) $(FIRMWARE)
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
