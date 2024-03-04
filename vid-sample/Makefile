MAKEFLAGS := --jobs=$(shell nproc)
#MAKEFLAGS +=--output-sync=target

OBJS=vid.o bpicam.o fbdisp.o
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -Wno-psabi -I../lib -L../lib
#CXXFLAGS+=-pg
CXXFLAGS+=-DDISPLAY=0
CXXFLAGS+=-DCAMSERVER=0
CXXFLAGS+=-O2

vid: $(OBJS)
	$(CXX) $(CXXFLAGS) -o vid $(OBJS) -l2702 -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lpthread -lntcore -lcscore -lcameraserver -lwpiutil -lapriltag -lwpimath

$(OBJS): v4lcam-mp.h bpicam.h fbdisp.h Makefile

