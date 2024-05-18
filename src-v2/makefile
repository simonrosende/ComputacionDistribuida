GXX = g++
CXXFLAGS = -g -std=c++11 `pkg-config --cflags eigen3 yaml-cpp gtkmm-3.0` -Wall -I.
LDFLAGS = `pkg-config --libs yaml-cpp gtkmm-3.0`
AR = ar 

SRC = simulation-plot.cc simulation.cc environment.cc swarm.cc robot.cc 

OBJ = $(SRC:.cc=.o)

%.o: %.cc %.hh
	$(GXX) $(CXXFLAGS) -o $@ -c $<

all: libMRS.a pairingSwarms
 
pairingSwarms: pairingSwarms.cc pairing.o
	$(GXX) $(CXXFLAGS) -I. -o $@ $^ -L. -lMRS $(LDFLAGS)

libMRS.a: $(OBJ)
	$(AR) rcs $@ $^
	ranlib $@

flocking.o: flocking.cc flocking.hh	
robot.o: robot.cc robot.hh
swarm.o: swarm.cc swarm.hh
environment.o: environment.cc environment.hh
boid.o: boid.cc boid.hh

clean:
	rm *.o *~ libMRS.a pairingSwarms
