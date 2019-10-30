CXX=g++
CPPFLAGS = -std=c++14 -Wall -Wextra -Werror -I/usr/include/mavsdk -I/usr/local/include/mavsdk
LDFLAGS = -L/usr/local/lib -lmavsdk -lmavsdk_action -lmavsdk_telemetry -lmavsdk_offboard

SRCS=src/main.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: ap_mav_control

%.o : %.cpp
	$(CXX) $(CPPFLAGS) -o $@ -c $<

ap_mav_control: $(OBJS)
	$(CXX) -o ap_mav_control $(OBJS) $(LDFLAGS)

clean:
	rm -f ap_mav_control $(OBJS)
.PHONY: clean
