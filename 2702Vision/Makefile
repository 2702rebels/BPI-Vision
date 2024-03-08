MAKEFLAGS := --jobs=$(shell nproc)

OBJS=2702Vision.o bpicam.o fbdisp.o
#OBJS=2702Vision.o bpicam.o fbdisp.o field.o rxfm.o PoseEstimator.o
#OBJS=2702Vision.o bpicam.o fbdisp.o 2702camera.o 2702math.o field.o
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -I../lib -L../lib -I../cminpack -L../cminpack -Wno-psabi
#CXXFLAGS+=-Wno-deprecated-enum-enum-conversion
CXXFLAGS+=-g
CXXFLAGS+=-O2
CXXFLAGS+=-DDISPLAY=0
CXXFLAGS+=-DCAMSERVER=0

2702Vision: ../cminpack/libcminpacks.a $(OBJS)
#	$(CXX) $(CXXFLAGS) -o 2702Vision $(OBJS) -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lwpiutil -lapriltag -lwpimath
#	$(CXX) $(CXXFLAGS) -o 2702Vision $(OBJS) -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lopencv_imgcodecs -lwpiutil -lapriltag -lwpimath -lcurses -l2702 -lcminpacks
	$(CXX) $(CXXFLAGS) -o 2702Vision $(OBJS) -l2702 -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lpthread -lntcore -lcscore -lcameraserver -lwpiutil -lapriltag -lwpimath -lcurses

#	$(CXX) $(CXXFLAGS) -o vid $(OBJS) -l2702 -lcameraserver -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lpthread -lntcore -lcscore -lcameraserver -lwpiutil -lapriltag -lwpimath

$(OBJS): v4lcam-mp.h bpicam.h ../lib/2702camera.h 2702Vision.h ../lib/2702math.h field.h Makefile rxfm.h

../cminpack/libcminpacks.a:
	(cd ../cminpack; make float)


clean:
	rm $(OBJS)
	rm 2702Vision

