MAKEFLAGS := --jobs=$(shell nproc)

OBJS=nttest.o
#OBJS=2702Vision.o bpicam.o fbdisp.o field.o rxfm.o PoseEstimator.o
#OBJS=2702Vision.o bpicam.o fbdisp.o 2702camera.o 2702math.o field.o
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -I../lib -L../lib -I../cminpack -L../cminpack -Wno-psabi
#CXXFLAGS+=-Wno-deprecated-enum-enum-conversion
CXXFLAGS+=-g
CXXFLAGS+=-O2
CXXFLAGS+=-DDISPLAY=0
CXXFLAGS+=-DCAMSERVER=1

nttest: $(OBJS)
	$(CXX) $(CXXFLAGS) -o nttest $(OBJS) -l2702 -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lpthread -lntcore -lcscore -lcameraserver -lwpiutil -lapriltag -lwpimath -lcurses


clean:
	rm $(OBJS)
	rm nttest

