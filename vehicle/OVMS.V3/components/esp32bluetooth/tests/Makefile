CXX      := g++
CXXFLAGS := -std=c++11 -Wall -Wextra -g -I../src

SRC := ../src/elm327_engine.cpp \
       test_elm327_engine.cpp

all: test

test: $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@
	./$@

clean:
	rm -f test

.PHONY: all test clean
