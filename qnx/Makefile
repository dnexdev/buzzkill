# Dev on macOS/Linux with default toolchain:
#   make
# QNX cross-build (once you have SDP + qcc in PATH):
#   make CXX=q++ CXXFLAGS="-Vgcc_ntoaarch64le -std=c++17 -Wall"
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
LDFLAGS  ?=

SRC := \
  src/main.cpp \
  src/protocol.cpp \
  src/receiver.cpp \
  src/calibration.cpp \
  src/tracker.cpp \
  src/servo_mock.cpp \
  src/servo_serial.cpp

OBJ := $(SRC:src/%.cpp=build/%.o)
BIN := build/buzzkill

all: $(BIN)

$(BIN): $(OBJ)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(OBJ) -o $@ $(LDFLAGS)

build/%.o: src/%.cpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build

.PHONY: all clean
