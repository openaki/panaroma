LAPACKLIBS=-llapack -lblas
CC=g++
CFLAGS=-c -Wall
LIBS=$(LAPACKLIBS) -lsba -lm \
-L/opt/opencv/lib -lcxcore -lcv -lhighgui -lcvaux -lml
LDFLAGS=-L../sba-1.5

SOURCES=MatchPair.cpp \
../ImageReg/BundleAdjust.cpp \
../ImageReg/Homography.cpp \
../ImageReg/Feature.cpp \
../ImageReg/SiftFeature.cpp \
../ImageReg/Camera.cpp

INCLUDE= \
-I../ImageReg \
-I../sba-1.5 \
-I/opt/opencv/include/opencv
# -I../ImageReg/BundleAdjust.h \
# -I../ImageReg/Homography.h \
# -I../ImageReg/Feature.h \
# -I../ImageReg/SiftFeature.h

OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=MatchPair

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)




