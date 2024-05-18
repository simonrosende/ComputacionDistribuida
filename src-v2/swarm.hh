/*
 * swarm.hh (C) Inaki Rano 2022
 * 
 * Declaration of the class Swarm for the 2D multi-robot simulator
 * 
 * The swarm class is derived from a standard vector of robot shared
 * pointers. The default way of filling in the vector is using the
 * "push_back()" method of the std::vector<>, however an alias has
 * been defined (method "add()") to fill the vector.
 *
 * The copy constructor makes a deep copy of the swarm, ans the
 * clone() method creates a new swarm also making a deep copy.
 *
 */
#ifndef MRS_SWARM_HH
#define MRS_SWARM_HH
#include <vector>

#include "robot.hh"

namespace mrs {
  // Forward declaration of the Swarm class
  class Swarm;

  // Swarm shared pointer type definition
  typedef std::shared_ptr<Swarm> SwarmPtr;

  // Swarm class definition, the class is a container (vector) of
  // robot shared pointers.
  class Swarm: public std::vector<RobotPtr> {
  public:
    // Default constructor empty vector
    Swarm() {}
    // Copy constructor makes a (deep) copy of the argument
    Swarm(const SwarmPtr & );
    // Destructor
    ~Swarm() {}

    // Add a robot to the swarm (just sets the pointer, not copied) it
    // just calls the push_back() method from the std::vector<> class
    void add(RobotPtr & rp) { push_back(rp); }

    // Clone the swarm and return a pointer to the new (deep) copied
    // swarm
    SwarmPtr clone() const;
    
    // Clone the robots with the specified IDs into a new swarm
    SwarmPtr clone(const std::vector<unsigned int> & robotId) const;

    // Overloaded operators to write the swarm into a stream
    friend std::ostream & operator<< (std::ostream & , const mrs::Swarm &);
    friend std::ostream & operator<< (std::ostream & , const mrs::SwarmPtr &);

  protected:
    // Simulation class needs to access the protected methods to
    // step() the simulation and create random swarms in the
    // environment using the protected version of the add() method.
    friend class Simulation;

    // Step the simulation, i.e. call the step() method for all the
    // robots in the swarm
    void step(double dt);

    // Add a new robot to the swarm. This is for the simulation class
    // to be able to generate random swarms.
    // Note: This function does not clone() the robot, so it must be a
    // self-standing shared pointer
    void add(RobotPtr & rp, unsigned int id, Position2d & pos);
    
  private:  
  };

  std::ostream & operator<< (std::ostream & , const mrs::Swarm &);
  std::ostream & operator<< (std::ostream & , const mrs::SwarmPtr &);
}


#endif
