# Compilador
CXX=g++


SOURCES=code.cpp


EXECUTABLE=spinningcube


CXXFLAGS=-std=c++11


LDLIBS=-losg -losgDB -losgViewer -losgGA


all: $(EXECUTABLE)


$(EXECUTABLE): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)


clean:
	rm -f *.o *~


cleanall: clean
	rm -f $(EXECUTABLE)

