/*
 * environment.hh (C) Inaki Rano 2022
 * 
 * Declaration of environment class for the 2D multi-robot simulator
 * 
 * The environment is where the swarm will be simulated. This
 * environment has a spherical topology, i.e. upper and lower lines
 * are the same as well as left and right ones. The main feature of
 * the environment are the position limits, which shoul be of the form
 * (-P,P) with P a 2D point (not sure if it would work otherwise)
 *
 * In these files three important functions are defined:
 *
 * - distance() Calculates the distance between two points. If the
 *   environment is not defined it returns the euclidean distance,
 *   otherwise it returns the distance accounting for the circular
 *   topology of the environment.
 *
 * - angle() Calculates the angle between the horizontal line (x axis)
 *   and the segment defined by two points. The segment goes from the
 *   first point to the second and it also considers the topology of
 *   the environment when it is defined.
 *
 * - wrap() Returns the position to the environment if it comes out on
 *   one of its borders.
 *
 */ 
#ifndef MRS_ENVIRONMENT_HH
#define MRS_ENVIRONMENT_HH
#include <vector>

#include "base.hh"
#include "robot.hh"
#include "swarm.hh"

namespace mrs {

  // Declaration of the class environment.
  class Environment {
  public:
    // Constructor
    Environment(const Position2d & min, const Position2d & max,
		bool isEuclidean = false);

    // Methods to get the limits of the environment.
    // Minimum bottom-left-most point
    const Position2d & min() const {return m_min;}
    // Maximum top-right-most point
    const Position2d & max() const {return m_max;}

    // Method to check a circular area in the environment is free
    // given a swarm.
    // The position is the centre of the circular area with radius "r"
    bool isFree(const Position2d &, double r = 0,
		const SwarmPtr swarm = nullptr) const;

    // Generate a random position in the environment
    Position2d random() const;
    
    // Generate a random position in the environment. The position
    // will be the centre of a free area of radius "r" given the
    // robots of the swarm. The method will try to generate points
    // "trials" times. If on return trials <=0 the position generated
    // is invalid, i.e. no free position was randomly found.
    Position2d random(float r, const SwarmPtr swarm, int & trials) const;

  protected:
    // External functions which need to access protected methods of
    // the environment class
    friend float distance(const Position2d &, const Position2d &);
    friend float angle(const Position2d &, const Position2d &);
    friend Position2d wrap(const Position2d &);
    friend Position2d average(const std::vector<Position2d> &);
    friend std::ostream & operator<< (std::ostream & , const mrs::Environment & );
    // Method to compute the distance within the environment taking
    // into account the topology
    float dist(const Position2d &, const Position2d &) const;
    // Method to compute the angle within the environment taking into
    // account the topology. The angle is relativa to the 'x' axis of
    // the environment and the segment is computed going from the
    // first point to the second, i.e. the first point is the 'origin'
    // of the reference system
    float ang(const Position2d &, const Position2d &) const;

    // Method to wrap a position outside the environment into the
    // environment.
    Position2d wrap(const Position2d &) const;

    // Singleton pattern, there must be only one environment to simulate.
    static Environment *m_envp;

  private:
    // Default constructor and copy constructors are deleted to
    // implement the singleton pattern (seems to be a cheap way)
    Environment() = delete;
    Environment(const Environment & e) = delete;
    
    Position2d m_min, m_max;
    Position2d m_size;
    bool m_isEuclidean;
  };

  // Overloaded operator to write the environment into a stream
  std::ostream & operator<< (std::ostream & , const mrs::Environment & );

}

#endif
