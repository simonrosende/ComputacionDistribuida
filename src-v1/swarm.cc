/*
 * swarm.hh (C) Inaki Rano 2022
 * 
 * Definition of the swarm class derived from the std::vector<> class
 *
 */
#include "swarm.hh"

namespace mrs {

  // Copy constructor makes a deep copy of the swarm calling the
  // clone() method of the robots in the original swarm
  Swarm::Swarm(const SwarmPtr & swarm)
  {
    for (Swarm::const_iterator iit = swarm->begin();
	 iit != swarm->end(); ++iit)
      push_back((*iit)->clone());
  }

  // Simulate the swarm calling the step() method. This method finds
  // all the neighbour robots to each robot in the swarm and creates a
  // vector of robots as the argument to call the action() method in
  // each of the robots. Then it invokes the step() method for the
  // robots in the swarm
  void
  Swarm::step(double dt)
  {
    // Quite inefficient method to make sure all robots in the swarm
    // get the old state, otherwise later robots in the vector might
    // perceive early robot at their new positions instead of the old
    // ones. This ensures all the robots perceive the swarm at the
    // same moment.
    SwarmPtr oldSwarm(clone());

    // Loop for all the robots in the swarm
    for (std::vector<RobotPtr>::iterator iit = begin();
	 iit != end(); ++iit)
      {
	RobotPtr rp(*iit);  // copy of the current robot pointer
	std::vector<RobotPtr> neighbours; // vector of near robots as
					  // perception for the
					  // current robot
	// Loop for all the robots in the original swarm to look for
	// those closer than rMax (perceptual range)
	for (std::vector<RobotPtr>::iterator jjt = oldSwarm->begin();
	     jjt != oldSwarm->end(); ++jjt)
	  {
	    RobotPtr np(*jjt); // Pointer to the candidate robot
	    // Add it to the neighbour list if it is in the perceptual
	    // range
	    if (rp->id() != np->id() &&
		distance(rp->position(), np->position()) < rp->settings().rMax)
	      neighbours.push_back(np);
	  }
	// Call the action method with the list of neighbour robots
	rp->action(neighbours);
	// Clear the list for the next robot
	neighbours.clear();
	// Call the step() method for the robot to move if necessary
	rp->step(dt);
      }
  }

  // Clone the swarm making a deep copy of the robots with Id in the
  // provided list
  SwarmPtr
  Swarm::clone(const std::vector<unsigned int> & robotId) const
  {
    SwarmPtr nSwarm = std::make_shared<Swarm>();

    for (std::vector<RobotPtr>::const_iterator iit = this->begin();
	 iit != this->end(); iit++)
      if (std::find(robotId.begin(), robotId.end(),
		    (*iit)->id()) != robotId.end())
	nSwarm->push_back((*iit)->clone());
    
    return nSwarm;
  }

  // Clone the whole swarm
  SwarmPtr
  Swarm::clone() const
  {
    // Make a vector with the IDs of all the robots in the swarm to
    // make a copy (this seems pretty inefficient to me, better
    // re-code this method)
    std::vector<unsigned int> ids;
    for (std::vector<RobotPtr>::const_iterator iit = this->begin();
	 iit != this->end(); iit++)
      ids.push_back((*iit)->id());

    return clone(ids);
  }
  
  // Add a robot to the swarm
  void
  Swarm::add(RobotPtr & rp, unsigned int id, Position2d & pos)
  {
    rp->position(pos);
    rp->id(id);
    push_back(rp);
  }


  std::ostream &
  operator<< (std::ostream & ofd, const mrs::Swarm & swarm)
  {
    ofd << "Swarm(" << swarm.size() << ")" << std::endl;
    for (unsigned int ii = 0; ii < swarm.size(); ii++)
      ofd << *swarm[ii];
    ofd << "============" << std::endl;
  
    return ofd;
  }

  std::ostream &
  operator<< (std::ostream & ofd, const mrs::SwarmPtr & swarmp)
  {
    ofd << "Swarm(" << swarmp->size() << ") [Ptr]" << std::endl;
    // for (mrs::Swarm::const_iterator iit = swarmp->begin();
    //      iit != swarmp->end(); iit++)
    //   ofd << **iit << std::endl;
  
    for (unsigned int ii = 0; ii < swarmp->size(); ii++)
      ofd << *(*swarmp)[ii];
    ofd << "============" << std::endl;
  
    return ofd;
  }
}
