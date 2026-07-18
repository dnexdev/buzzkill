# Native build on the QNX Pi target.
#   sudo apk add qnx-gcc qnx-sensor-framework-dev   # already installed
#   make
#
# Produces ./camsrc which streams BGR frames on stdout.
# See camsrc.cpp for the wire format.

CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
LDFLAGS  ?=
LDLIBS   ?= -lcamapi

BIN := camsrc
SRC := camsrc.cpp

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(BIN)

.PHONY: all clean
