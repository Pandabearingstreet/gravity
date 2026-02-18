CXX = g++
CXXFLAGS = -Wall -Wextra

# default target
all: main

main: main.cpp
	$(CXX) $(CXXFLAGS) -o main main.cpp -lncurses

