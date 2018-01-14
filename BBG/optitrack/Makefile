SRC_PATH 	:= $(shell pwd)/../.
BIN_PATH	:= $(SRC_PATH)/bin/.
COMMON_LIB_PATH := $(SRC_PATH)/optitrack/common/lib/.
LCMTYPES_PATH	:= $(SRC_PATH)/../lcmtypes/. 

CXX		:= g++

CFLAGS_SHARED	:= -g -DFILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_REENTRANT \
			-Wall -Wno-unused-parameter -Wno-deprecated-declarations \
			-pthread -fPIC -I $(SRC_PATH)	

CFLAGS_STD	:= -std=gnu99 -Wno-format-zero-length $(CFLAGS_SHARED)
CXXFLAGS_STD 	:= -std=c++0x $(CFLAGS_SHARED) 
CFLAGS_LCM	:= -I $(LCMTYPES_PATH) -I `pkg-config --cflags lcm`

LDFLAGS_STD	:= -lm -lpthread
LDFLAGS_COMMON	:= -L -lcommon $(COMMON_LIB_PATH)/libcommon.a
LDFLAGS_LCM	:= -L -llcmtypes `pkg-config --libs lcm`

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_LCM) \
	-O3
	
LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCM) 

BIN_OPTITRACK_DRIVER=$(BIN_PATH)/optitrack_driver

all:  $(BIN_OPTITRACK_DRIVER)

# Make optitrack executables
$(BIN_OPTITRACK_DRIVER): optitrack.o optitrack_driver.o
	@echo "    $@"
	@$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

# Clean target
clean:
	@rm -f *~ *.o $(BIN_OPTITRACK_DRIVER) *.a
