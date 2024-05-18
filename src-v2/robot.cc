/*
 * robot.cc (C) Inaki Rano 2022
 * 
 * Implementation of the class Robot, basic virtual class to further
 * define other robots.
 */
#include "robot.hh"


namespace mrs {
  // Function to wrap the position of the robot in environments with
  // circular topology
  extern Position2d wrap(const Position2d &);

  // Default settings of a robot
  RobotSettings defaultRobotSettings = {0.2,      // Size
					2.5,      // Perception range
					0.0,      // Min. speed
					0.5,      // Max. speed
					0.0,      // Min. w
					2 * M_PI, // Max. w
					true,     // Visible
					{0.0,0.0,0.0}}; // Color (black)

  // Method to get the velocity of the robot, in a typical implementation
  const Velocity2d &
  Robot::action(std::vector<RobotPtr> & per) 
  {	
    
    return m_vel; // should be a zero vector in this robot class
  }
  
  // Deep copy of the robot class and returns a shared pointer
  RobotPtr
  Robot::clone() const
  {
    // Create a new robot object and return a shared pointer to it
    RobotPtr newRobot = std::make_shared<Robot>(m_id, m_pos, m_settings);
    
    return newRobot;
  }

  // Simulate the robot motion given the current robot velocity 
  void
  Robot::step(float dt)
  {

    // Calculate the norm of the velocity (speed)
    float v(m_vel.norm());
    
    // Velocity will be zero if the speed is smaller than vMin
    if (v < m_settings.vMin)
      {
	m_vel = Velocity2d::Zero();
	return;
      }
    // Scale velocity if the speed is larger than vMax, the velocity
    // will have speed vMax
    Velocity2d realV(m_vel);
    if (v > m_settings.vMax)
      realV *= (m_settings.vMax / v);
    // Update the position of the robot according to its velocity
    // using Euler method
    m_pos += dt * realV;

    // Wrap the position if the topology of the environment is not
    // planar
    m_pos = wrap(m_pos);
  }

  // Print all the robot information on a stream
  std::ostream &
  operator<< (std::ostream & ofd, const mrs::Robot & r)
  {
    ofd << "R-" << r.id()
	<< " name: " << r.name()
	<< " x: " << r.position()[0] << " y: " << r.position()[1]
	<< " Settings: {" <<  r.settings().radius << ", " <<  r.settings().rMax << ", "
	<<  r.settings().vMin << ", " <<  r.settings().vMax << ", "
	<<  r.settings().wMax << ", "
	<<  r.settings().visible << ", ["
	<< int(r.settings().color[0]) << ","
	<< int(r.settings().color[1]) << ","
	<< int(r.settings().color[2]) << "] } "
	<< " Params: [";
    for (unsigned int ii = 0; ii < r.parameterVector().size() ;ii++)
      ofd << r.parameterVector()[ii]
	  << (ii+1 == r.parameterVector().size() ? "" : ",");
    ofd << "]" << std::endl;
  
    return ofd;  
  }

  // Print all the robot information on a stream
  std::ostream &
  operator<< (std::ostream & ofd, const mrs::RobotPtr & rp)
  {
    ofd << "R-" << rp->id()
	<< " name: " << rp->name()
	<< " x: " << rp->position()[0] << " y: " << rp->position()[1]
	<< " Settings: {" <<  rp->settings().radius << ", " <<  rp->settings().rMax << ", "
	<< rp->settings().vMin << ", " <<  rp->settings().vMax << ", "
	<< rp->settings().wMax << ", "
	<< rp->settings().visible << ", ["
	<< int(rp->settings().color[0]) << ","
	<< int(rp->settings().color[1]) << ","
	<< int(rp->settings().color[2]) << "] } "
	<< " Params: [";
    for (unsigned int ii = 0; ii < rp->parameterVector().size() ;ii++)
      ofd << rp->parameterVector()[ii]
	  << (ii+1 == rp->parameterVector().size() ? "" : ",");
    ofd << "]" << std::endl;
  
    return ofd;  
  }


}




