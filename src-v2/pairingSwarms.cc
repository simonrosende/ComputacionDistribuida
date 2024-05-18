#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <gtkmm.h>


#include <robot.hh>
#include <swarm.hh>
#include <environment.hh>
#include <simulation.hh>
#include <simulation-plot.hh>


#include "pairing.hh"

// Create an environment for the swarm
mrs::Environment env(mrs::Position2d(-20,-20), mrs::Position2d(20,20));

int
main()
{
  // TODO: You can change the settings of the robots
  // Settings of the robots, different than the default settings
  mrs::RobotSettings rSettings = {0.15,     // Robot radius
				  2.0,      // Perceptual range
				  0.0,      // Min speed
				  0.75,     // Max speed
				  0.0,      // Min Omega
				  1.5,      // Max Omega
				  true,     // Visible
				  {1,0,0}}; // Colour
  
  

  // TODO: You can change the number of robots in the
  // environment. Notice that it might not be possible to place all
  // the robots in the environment if there is no space left.
  // Create a random swarm of 20 robots
  const int numberOfRobots(20);
  // Initialise random number generator
  srand((unsigned int) time(0));

  
  // Create a random set of robots with two different colors
  mrs::SwarmPtr swarmp = std::make_shared<mrs::Swarm>();

  // Red robots
  rSettings.radius = 0.15;
  for (unsigned int ii = 0 ; ii < numberOfRobots / 2; ii++) {
    int trials(100);
    mrs::Position2d pt = env.random(rSettings.radius, swarmp, trials);
    if (trials <= 0)
      std::cerr << "Error creating random swarm" << std::endl;
    mrs::Velocity2d vel(mrs::Velocity2d::Random());
    vel *= rSettings.vMax / vel.norm();
    mrs::RobotPtr rp = std::make_shared<PairingRobot>(ii, pt, rSettings,
						      vel);
    swarmp->push_back(rp);
  }

  // Blue robots
  rSettings.color[0] = 0;
  rSettings.color[2] = 1;
  for (unsigned int ii = 0 ; ii < numberOfRobots / 2; ii++) {
    int trials(100);
    mrs::Position2d pt = env.random(rSettings.radius, swarmp, trials);
    if (trials <= 0)
      std::cerr << "Error creating random swarm" << std::endl;
    mrs::Velocity2d vel(mrs::Velocity2d::Random());
    vel *= rSettings.vMax / vel.norm();    
    mrs::RobotPtr rp = std::make_shared<PairingRobot>(numberOfRobots / 2+ii, pt, rSettings,
						      vel);
    swarmp->push_back(rp);
  }
  
  // Create a Simulation object to simulate the swarm
  mrs::SimulationPtr simp(std::make_shared<mrs::Simulation>(swarmp));
  simp->tMax(300);
  
  // Run the simulation
  simp->run();

  // Uncomment this if you want to inspect the trajectory as a text
  // file (see header file simulation.hh to check for the format)
  //simp->saveTraj("trajectory.txt");
  
  // Show the animation of the retulting swarm trajectory
  Glib::RefPtr<Gtk::Application>  app = Gtk::Application::create("es.usc.mrs");
  mrs::SimulationWindow dsp(simp, env);
  app->run(dsp);
  
  return 0;
}
