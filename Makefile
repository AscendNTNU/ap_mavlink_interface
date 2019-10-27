CXX=g++
CPPFLAGS = -std=c++14 -Wall -Wextra -Werror -Iinclude -I/usr/include/mavsdk
LDFLAGS = -lmavsdk -lmavsdk_action -lmavsdk_telemetry -lmavsdk_offboard

SRCS=src/main.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: ap_mav_control

%.o : %.cpp
	$(CXX) $(CPPFLAGS) -o $@ -c $<

ap_mav_control: $(OBJS)
	$(CXX) $(LDFLAGS) -o ap_mav_control $(OBJS)

clean:
	rm -f ap_mav_control $(OBJS)
.PHONY: clean
