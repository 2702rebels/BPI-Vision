OBJS=log2mjpeg.o
#CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -Wno-psabi -fopenmp -Wno-deprecated-enum-enum-conversion
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -Wno-psabi -Wno-deprecated-enum-enum-conversion -I../lib -L../lib
CXXFLAGS+=-g
#CXXFLAGS+=-O2

log2mjpeg: $(OBJS)
	$(CXX) $(CXXFLAGS) -o log2mjpeg $(OBJS) -l2702 -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lwpiutil

#$(OBJS): v4lcam-mp.h bpicam.h

