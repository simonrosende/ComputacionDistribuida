/*
 * simulation-plot.hh (C) Inaki Rano 2022
 * 
 * Declaration of the classes to display a simulation.
 *
 * This classes depend on gtkmm 3.0 and are a lousy implementation of
 * a GUI to display the simulation. 
 *
 * The window supports the following commands:
 * Arrow Up/Arrow Right: Speed up the speed of plotting the simulation
 * Arrow Down/Arrow Left: Slow down the speed of plotting the simulation
 * Spacebar: Start/Stop the simulation plotting
 * Escape Key: Quit the program
 *
 * The main class to plot a simulation is the SimulationWindow, which
 * needs as arguments to the constructor a pointer to the Simulation
 * object and a reference to the environment.
 */
#ifndef MRS_SIMULATION_PLOT_HH
#define MRS_SIMULATION_PLOT_HH

#include <gtkmm.h>

#include "environment.hh"
#include "simulation.hh"

namespace mrs {
  class SimulationPlot;
  
  class SimulationDraw : public Gtk::DrawingArea {
  public:
    SimulationDraw(const SimulationPtr &, const Position2d & ,
		   const Position2d & );
    virtual ~SimulationDraw();
    
  protected:
    friend class SimulationWindow;
    void draw_swarm(float t, float speed);
    
    bool on_draw(const Cairo::RefPtr<Cairo::Context> & cr) override;
    
  private:
    SimulationPtr m_simulationPtr;
    float m_t;
    float m_speed;
    Position2d m_size;
    Position2d m_min;
    Position2d m_max;
  };
  
  class SimulationWindow : public Gtk::Window {
  public:
    SimulationWindow(const SimulationPtr & , const Environment &);

  private:
    float computeSpeed();
    void timeout(void);
    bool draw_swarm(const Cairo::RefPtr<Cairo::Context> & cr);
    bool key_pressed(GdkEventKey * );
    
    SimulationDraw m_simulationDraw;
    char m_speed;
    char m_maxSpeed;
    bool m_playStatus;
    int m_update_ms;
    float m_base_ms;
  };
  
}
#endif
