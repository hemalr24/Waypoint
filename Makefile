CXX ?= g++
CPPFLAGS += -Iinclude
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic

.PHONY: all test demo compare
all: build/simulate

build:
	mkdir -p build

build/simulate: src/controller.cpp src/simulate.cpp include/route_robot/controller.hpp include/route_robot/simulation.hpp include/route_robot/odometry.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/controller.cpp src/simulate.cpp -o $@

build/controller_tests: src/controller.cpp tests/controller_tests.cpp include/route_robot/controller.hpp include/route_robot/simulation.hpp include/route_robot/odometry.hpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) src/controller.cpp tests/controller_tests.cpp -o $@

test: build/controller_tests build/simulate
	./build/controller_tests
	python3 -m unittest discover -s tests -p 'test_*.py'

demo: all
	python3 scripts/run_experiments.py --animate

compare: all
	python3 scripts/compare_feedback.py
