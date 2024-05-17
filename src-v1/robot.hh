/*
 * robot.hh (C) Inaki Rano 2022
 * 
 * Definition of the base class Robot for the 2D multi-robot
 * simulator.  Specific implementations of a robot in multirobot
 * systems should be derived from this class.
 * 
 * The robot follows an integrator model and can perceive other robots
 * in the environment within the perception range. Specifically it can
 * perceive them as a standard vector of robots and therefore it get
 * access to all their features, such as position, size, color...
 *
 * This type of robots can be used to define static objects,
 * i.e. obstacles.
 */
#ifndef MRS_ROBOT_HH
#define MRS_ROBOT_HH
#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include "base.hh"

namespace mrs {

  // Forward class declaration to define the pointer to a robot (smart
  // pointer)
  class Robot;
  
  // Robot shared pointer type definition. This allows to create new
  // robots using:
  //
  // std::make_shared<Robot>([constructor params])
  //
  // For derived classes the type of object to allocate must be
  // different, but a RobotPtr can still be used
  typedef std::shared_ptr<Robot> RobotPtr;

  //! Configuration and capabilities of the robot
  // 
  //  - radius: radius of the robot for plotting and collision
  //            purposes (note collisions are not detected yet)
  //  - rMax: Perception range of the robot
  //  - vMin: Minimum speed (speeds set under this value will
  //          become zero)
  //  - vMax: Maximum velocity (speeds larger than this will be
  //          clamped)
  //  - wMin: Not used
  //  - wMax: Not used
  //  - visible: Not used
  //  - color: Color of the robot for plotting and detection purposes
  typedef struct {
    float radius; // Size
    float rMax;   // Perception range
    float vMin;   // Minimum speed (dead-zone)
    float vMax;   // Maximum speed
    float wMin;   // Minimum rotation speed (dead-zone)
    float wMax;   // Maximum rotation speed
    bool visible;  // Is the robot visible 
    float color[3]; // Colour of the robot
  } RobotSettings;

  // Default configuration to create robots
  // - radius: 0.2 m
  // - rMax: 2.5 m
  // - vMin: 0
  // - vMax: 0.5 m/s
  // - color: [0,0,0] (black)
  extern RobotSettings defaultRobotSettings;
  
  /*! 
   * This class defines the basic interface for a robot.
   *
   * The robot follows a single integrator model in both x and y
   * coordinates. The computed velocity of the robot is stored in the
   * class together with its position.
  */
  class Robot {
  public:
    /* 
     * Robot constructor parameters
     * - id: Unique ID of the robot (used to avoid self-perception)
     * - p: Robot position
     * - settings: Robot settings (defautRobotSettings by default, see
     *    above)
     */
    Robot(unsigned int id, const Position2d & p, 
	  const RobotSettings & settings = defaultRobotSettings):m_id(id),
								 m_pos(p),
								 m_vel(Velocity2d::Zero()),
								 m_settings(settings)
    {}
    
    /*
     * Virtual destructor does nothing
     */
    virtual ~Robot() {}
    
    //! Get the robot ID
    unsigned int id() const {return m_id;}
    
    // Unique name for each robot type
    virtual std::string name() const { return std::string("none"); }

    // Parameters of the robot as a std::vector<float> in case it has
    // parameters
    virtual std::vector<float> parameterVector() const
    { return std::vector<float>(); }
    
    // Get the robot settings
    const RobotSettings & settings() const {return m_settings;}    

    void color(float r, float g, float b)
    {
      m_settings.color[0] = r;
      m_settings.color[1] = g;
      m_settings.color[2] = b;
    }

    bool sameColor(const RobotPtr & r)
    { return ((settings().color[0] == r->settings().color[0]) &&
	      (settings().color[1] == r->settings().color[1]) &&
	      (settings().color[2] == r->settings().color[2]));}
    
    // Get the position of the robot
    const Position2d & position() const { return m_pos; }

    // Get the velocity of the robot (must be set by calling action()
    // method)
    const Velocity2d & velocity() const { return m_vel; }

    // Compute the velocity of the robot based on the list of nearby
    // robots. This method is virtual to be able to redefine it for
    // different robots in the swarm, but mit *must* set the velocity
    // in the corresponding variable of the class (m_vel) and return
    // it. *Do not* return the computed velocity without setting it or
    // the robot won't move upon calling the step() method.
    virtual const Velocity2d & action(std::vector<RobotPtr> & );
    
    //! Create a (deep) copy of the agent and return the shared
    //! pointer it might need to be redefined for derived classes
    //! (virtual)
    virtual RobotPtr clone() const;
    
    // Overloaded operators to print the robot information on a stream
    friend std::ostream & operator<< (std::ostream & , const mrs::Robot &);
    friend std::ostream & operator<< (std::ostream & , const mrs::RobotPtr &);
    
  protected:
    friend class Swarm;
    
    // Simulate the motion of the robot with a step size dt
    void step(float dt);
    
    // To set the position of the robot in the Swarm class
    void position(Position2d p) { m_pos = p; }

    // To set the position of the robot in the Swarm class
    void id(unsigned int id) { m_id = id; }


    unsigned int m_id;          // Robot ID
    Position2d m_pos;           // Robot position
    Velocity2d m_vel;           // Robot velocity
    RobotSettings m_settings;   // Robot settings
  };

  std::ostream & operator<< (std::ostream & , const mrs::Robot & );
  std::ostream & operator<< (std::ostream & , const mrs::RobotPtr & );
}


#endif
