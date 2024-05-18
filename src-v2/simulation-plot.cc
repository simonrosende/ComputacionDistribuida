/*
 * simulation-plot.hh (C) Inaki Rano 2022
 * 
 * Definition of the classes to display a simulation.
 *
 */
#include <stdlib.h>
#include <sstream>
#include <iomanip>

#include <iostream>    

#include "simulation-plot.hh"

namespace mrs {

  SimulationDraw::SimulationDraw(const SimulationPtr &sim,
				 const Position2d &min,
				 const Position2d & max)
  {
    //m_cairoContext = get_window()->create_cairo_context();
    m_simulationPtr = sim;
    m_min = min;
    m_max = max;
    m_size = Position2d(600, 600);
    m_t = 0.0;
    m_speed = 1;
  }

  SimulationDraw::~SimulationDraw()
  {}

  void
  SimulationDraw::draw_swarm(float dt, float speed)
  {
    m_t += dt;
    m_speed = speed;
  }
  
  bool
  SimulationDraw::on_draw(const Cairo::RefPtr<Cairo::Context> & cr)
  {    
    Gtk::Allocation allocation = get_allocation();
    m_size[0] = (float)allocation.get_width();
    m_size[1] = (float)allocation.get_height();
    
    Position2d scale(m_size.cwiseQuotient(m_max - m_min));
    Position2d c(m_size/2);
    SwarmPtr swarmp = (*m_simulationPtr)[m_t];
    
    cr->save();
    Pango::FontDescription font;

    font.set_family("Monospace");
    font.set_weight(Pango::WEIGHT_BOLD);
    std::stringstream stream;
    stream << "Play Speed: "
      << std::fixed << std::setprecision(2) << m_speed
      << " Time: " << m_t;

    auto layout = create_pango_layout(stream.str().c_str());
    layout->set_font_description(font);
    cr->move_to(0,0);
    layout->show_in_cairo_context(cr);
    cr->stroke();
    
    for (unsigned int ii = 0; ii < swarmp->size(); ii++)
      {
	
	RobotPtr rp((*swarmp)[ii]);
	double r(scale.minCoeff() * rp->settings().radius);
	cr->set_source_rgba(rp->settings().color[0], rp->settings().color[1],
			    rp->settings().color[2], 1.0);
	
	Position2d fc(2*ii,-2*(float)ii);
	Position2d flip(1, -1);
	Position2d rc(m_size/2 +
		      scale.cwiseProduct(flip.cwiseProduct(rp->position())));
	
	cr->arc(rc[0], rc[1], r, 0.0, 2.0 * M_PI); // full circle
	cr->fill_preserve();
	cr->stroke();
      }
    cr->restore();  // back to opaque black

    return true;
  }
  
  
  SimulationWindow::SimulationWindow(const SimulationPtr & sim,
				     const Environment & env):
    m_simulationDraw(sim, env.min(), env.max())
  {
    set_title("Multi Robot Simulator");
    set_default_size(600, 600);
    set_resizable(true);
    signal_key_press_event().connect(sigc::mem_fun(*this, 
						   &SimulationWindow::key_pressed), false);
    add(m_simulationDraw);
    m_simulationDraw.show();
    m_speed = 0;
    m_base_ms = 1000 * sim->dt();
    m_maxSpeed = 9;
    m_playStatus = false;
    m_update_ms = 0;
    int update_ms(1000);
    Glib::signal_timeout().connect_once(sigc::mem_fun(*this,
						      &SimulationWindow::timeout),
					update_ms);
  }

  bool
  SimulationWindow::key_pressed(GdkEventKey * k)
  {
    switch (k->keyval) {
    case GDK_KEY_Up:
      m_speed++;
      m_speed = (m_speed > m_maxSpeed) ? m_maxSpeed : m_speed;
      break;
    case GDK_KEY_Right:
      m_speed++;
      m_speed = (m_speed > m_maxSpeed) ? m_maxSpeed : m_speed;
      break;
    case GDK_KEY_Down:
      m_speed--;
      m_speed = (m_speed < -m_maxSpeed) ? -m_maxSpeed : m_speed;
      break;
    case GDK_KEY_Left:
      m_speed--;
      m_speed = (m_speed < -m_maxSpeed) ? -m_maxSpeed : m_speed;
      break;
    case GDK_KEY_space:
      m_playStatus = !m_playStatus;
      break;
    case GDK_KEY_Escape:
      exit(0);
      break;
    }

    return true;
  }

  float
  SimulationWindow::computeSpeed()
  {
    if (m_speed == 0)
      return 1.0;
    if (m_speed > 0)
      return ((float)(1 + m_speed));
    
    return 1/((float)(1 - m_speed));
  }


  void
  SimulationWindow::timeout()
  {

    if (m_playStatus)
      {
	m_simulationDraw.draw_swarm(float(m_base_ms) / 1000, computeSpeed());
	get_window()->invalidate(true);
      }
    Glib::signal_timeout().connect_once(sigc::mem_fun(*this,
						      &SimulationWindow::timeout),
					m_update_ms);

    m_update_ms = int(m_base_ms / computeSpeed());
    
  }
  
}
