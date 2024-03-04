OBJS=Bayer2RGB.o
#CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -Wno-psabi -fopenmp
CXXFLAGS+=-std=c++20 -I/usr/local/frc/include -L/usr/local/frc/lib -I/usr/local/frc/include/opencv4 -Wno-psabi
CXXFLAGS+=-g
#CXXFLAGS+=-O2

vid: $(OBJS)
	$(CXX) $(CXXFLAGS) -o Bayer2RGB $(OBJS) -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lwpiutil

#$(OBJS): v4lcam-mp.h bpicam.h

