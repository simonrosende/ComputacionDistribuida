/*
 * simulation.cc (C) Inaki Rano 2022
 * 
 * Definition of the simulation class for the 2D multi-robot simulator
 *
 *
 *
 */
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "simulation.hh"

namespace mrs {

  
  Simulation::Simulation(const SwarmPtr & swarm): m_dt(0.1), m_tMax(100),
						  m_swarm_t0(swarm->clone())
  {}
  
  SwarmPtr
  Simulation::rndSwarm(unsigned int sz, const RobotPtr & rp,
		       const Environment & env)
  {
    SwarmPtr swarm = std::make_shared<Swarm>();
    int maxTrials(50 * sz);
    int idx(0);
    do {
	int trials(200);
	Position2d p(env.random(rp->settings().radius, swarm, trials));
	if (trials > 0)
	  {
	    RobotPtr nrp = rp->clone();
	    swarm->add(nrp, idx, p);
	    idx++;
	  }
	maxTrials--;
      }
    while ((maxTrials > 0) && (idx < sz));
    
    return swarm;
  }

  void
  Simulation::run()
  {
    if (m_swarm_t0 == nullptr) {
      m_traj.clear();
      return;
    }
    
    float t(0);
    m_traj.push_back(m_swarm_t0);
    while (t < m_tMax) {
      std::cout << "Time: " << t << std::endl;
      SwarmPtr nSwarm(m_traj.back()->clone());
      nSwarm->step(m_dt);
      m_traj.push_back(nSwarm);
      t += m_dt;
    }
    return ;
  }
  
  void
  Simulation::save(const char * filename) const
  {
    YAML::Node file;
    file["name"] = "MRS";
    file["version"] = "v-0.1-beta";
    file["dt"] = m_dt;
    file["tMax"] = m_tMax;
    YAML::Node swarm;
    swarm["size"] = m_swarm_t0->size();
    for (unsigned int ii = 0; ii < m_swarm_t0->size(); ii++)
      {
	YAML::Node robot;
	robot["id"] = (*m_swarm_t0)[ii]->id();
	robot["name"] = (*m_swarm_t0)[ii]->name();
	YAML::Node settings;
	settings["radius"] = (*m_swarm_t0)[ii]->settings().radius;
	settings["rMax"] = (*m_swarm_t0)[ii]->settings().rMax;
	settings["vMin"] = (*m_swarm_t0)[ii]->settings().vMin;
	settings["vMax"] = (*m_swarm_t0)[ii]->settings().vMax;
	settings["wMin"] = (*m_swarm_t0)[ii]->settings().wMin;
	settings["wMax"] = (*m_swarm_t0)[ii]->settings().wMax;
	settings["visible"] = (*m_swarm_t0)[ii]->settings().visible;
	YAML::Node color;
	color.push_back(int((*m_swarm_t0)[ii]->settings().color[0]));
	color.push_back(int((*m_swarm_t0)[ii]->settings().color[1]));
	color.push_back(int((*m_swarm_t0)[ii]->settings().color[2]));
	settings["color"] = color;
	robot["settings"] = settings;
	YAML::Node params;
	for (unsigned int jj = 0; jj < (*m_swarm_t0)[ii]->parameterVector().size(); jj++)
	  params.push_back((*m_swarm_t0)[ii]->parameterVector()[jj]);
	robot["parameters"] = params;
	
	swarm["robot"].push_back(robot);
	
      }
    file["swarm"] = swarm;
    
    YAML::Node traj;
    for (unsigned int ii = 0; ii < m_traj.size(); ii++)
      {
	YAML::Node step;
	step["t"] = ii * m_dt;
	YAML::Node coords;
	for (unsigned int jj = 0; jj < m_traj[ii]->size(); jj++)
	  {
	    coords.push_back((*m_traj[ii])[jj]->position()[0]);
	    coords.push_back((*m_traj[ii])[jj]->position()[1]);
	  }
	step["coordinates"] = coords;
	traj.push_back(step);
      }
    file["trajectory"] = traj;
    
    std::ofstream ofd(filename);
    ofd << file;
    ofd.close();
  }

  void
  Simulation::save(const std::string & filename) const
  {
    save(filename.c_str());
  }

  void
  Simulation::saveTraj(const char * filename) const
  {
    std::ofstream ofd(filename);
    for (unsigned int ii = 0; ii < m_traj.size(); ii++)
      {
	ofd << ii * m_dt << " ";
	SwarmPtr swarmp = m_traj[ii];
	for (unsigned int jj = 0; jj < swarmp->size(); jj++)
	  {
	    RobotPtr r = (*swarmp)[jj];
	    ofd << r->position()[0] << " " << r->position()[1] << " ";
	  }
	ofd << std::endl;
      }
    ofd.close();
  }
  
  void
  Simulation::load(const char * filename)
  {
    YAML::Node file(filename);
  }
  
  void
  Simulation::load(const std::string & filename)
  {
    load(filename.c_str());
  }
}
