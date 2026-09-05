# Plain-make build, so a fresh clone works without CMake installed.
# `make` builds both binaries, `make test` runs the suite.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude
BUILD    := build

SRC := src/driver_index.cpp

.PHONY: all run demo test bench sweep asan clean

all: $(BUILD)/unit_tests $(BUILD)/bench $(BUILD)/demo

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/unit_tests: $(SRC) tests/test_driver_index.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/bench: $(SRC) bench/benchmark.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/demo: $(SRC) examples/demo.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Build everything, verify it, then show it working. One command, start to end.
run: all
	@echo "=== 1/3  tests ==============================================="
	@./$(BUILD)/unit_tests
	@echo
	@echo "=== 2/3  dispatch simulation ================================="
	@./$(BUILD)/demo --ticks 45 --drivers 5000 --riders 4
	@echo
	@echo "=== 3/3  benchmark ==========================================="
	@./$(BUILD)/bench --drivers 100000 --queries 50000 --updates 50000

demo: $(BUILD)/demo
	./$(BUILD)/demo

test: $(BUILD)/unit_tests
	./$(BUILD)/unit_tests

bench: $(BUILD)/bench
	./$(BUILD)/bench

sweep: $(BUILD)/bench
	./$(BUILD)/bench --sweep

# Run the tests under the sanitizers before claiming correctness.
asan: | $(BUILD)
	$(CXX) -std=c++17 -O1 -g -fsanitize=address,undefined -Iinclude \
	  $(SRC) tests/test_driver_index.cpp -o $(BUILD)/unit_tests_asan
	./$(BUILD)/unit_tests_asan

clean:
	rm -rf $(BUILD)
