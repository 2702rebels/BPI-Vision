MAKEFLAGS := --jobs=$(shell nproc)

CXXFLAGS=-I/usr/local/frc/include -I/usr/local/frc/include/opencv4
CXXFLAGS+=-O2

OBJS=VisLog.o Bayer2RGB.o 2702camera.o 2702math.o genio.o Bayer2Grey.o

lib2702.a: $(OBJS)
	ar -r lib2702.a $(OBJS)
