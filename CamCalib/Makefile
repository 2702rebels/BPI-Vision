MAKEFLAGS := --jobs=$(shell nproc)

CXXFLAGS=-g -I../cminpack -L../cminpack -I/usr/local/frc/include/opencv4 -L/usr/local/frc/lib
#OBJS=camcalib.o 2702camera.o 2702math.o stereoCalib.o monoCalib.o
OBJS=calib.o 2702camera.o 2702math.o stereoCalib.o monoCalib.o rxfm.o

calib: $(OBJS) cminpacks 
	$(CXX) $(CXXFLAGS) -o calib $(OBJS) -lncurses -lcminpacks -lopencv_core

#	$(CXX) $(CXXFLAGS) -o vid $(VOBJS) -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpthread -lntcore -lcscore -lcameraserver -lwpiutil

#2702math.o: ../Raspberry\ Pi\ Program/2702math.cpp
#	$(CXX) $(CXXFLAGS) -c -o $@ ../Raspberry\ Pi\ Program/$*.cpp
#
#2702camera.o: ../Raspberry\ Pi\ Program/2702camera.cpp
#	$(CXX) $(CXXFLAGS) -c -o $@ ../Raspberry\ Pi\ Program/$*.cpp

cminpacks:
	(cd ../cminpack; make float)

$(OBJS): stereoCalib.h rxfm.h RawFile.h
