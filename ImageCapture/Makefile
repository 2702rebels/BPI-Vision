MAKEFLAGS := --jobs=$(shell nproc)

OBJS=vid.o bpicam.o fbdisp.o genio.o
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -Wno-psabi
#CXXFLAGS+=-pg
#CXXFLAGS+=-g
CXXFLAGS+=-O2

vid: $(OBJS)
	$(CXX) $(CXXFLAGS) -o vid $(OBJS) -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lwpiutil -lapriltag -lwpimath -lcurses

$(OBJS): v4lcam-mp.h bpicam.h

