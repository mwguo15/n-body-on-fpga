# Compiler and flags
CXX      = g++
CXXFLAGS = -std=c++11 -Wall -O2 -Iinclude

# Target executable name and source files
TARGET   = raytracer
SRC      = main.cpp raytracer.cpp

# Automatically generate object file names from source files
OBJ      = $(SRC:.cpp=.o)

# Default target: build the executable
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJ)

# Rule to compile .cpp files into .o object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up generated files
clean:
	rm -f $(TARGET) $(OBJ)

.PHONY: all clean
