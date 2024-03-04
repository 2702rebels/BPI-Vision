MAKEFLAGS := --jobs=$(shell nproc)

OBJS=vid.o bpicam.o
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -I../lib -L../lib
#CXXFLAGS+=-g
CXXFLAGS+=-O2

vid: $(OBJS)
	$(CXX) $(CXXFLAGS) -o vid $(OBJS) -l2702 -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lwpiutil

$(OBJS): v4lcam-mp.h bpicam.h

