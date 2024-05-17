/*
 * base.hh (C) Inaki Rano 2022
 * 
 * Definition of types for the 2D multi-robot simulator
 * 
 * Types defined (from Eigen):
 * - Position2d
 * - Velocity2d
 *
 * Functions to wrap variables with circular topology, useful for
 * environments with spherical or cylindrical topologies: 
 *
 * - wrapVar(): Wraps an individual variable
 * - wrap2pi(): Wraps an angular variable to the range (-pi,pi]
 *
 */
#ifndef MR_BASE_HH
#define MR_BASE_HH
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

// 4Debug
#define deg2rad(x) ((x) * M_PI / 180)
#define rad2deg(x) ((x) * 180 / M_PI)  
// 4DebugEnds

namespace mrs {
  
  typedef Eigen::Vector2f Position2d;
  typedef Eigen::Vector2f Velocity2d;

  float distance(const Position2d &, const Position2d &);
  float angle(const Position2d &, const Position2d &);
  Position2d wrap(const Position2d &);
  Position2d average(const std::vector<Position2d> &);
  float averageAngle(const std::vector<float> &);
  float averageAngleW(const std::vector<float> &, const std::vector<float> &);
  
  inline float wrapVar(float z, float size)
  {
    while (z <= -(size / 2)) z += size;
    while (z > (size / 2)) z -= size;
    
    return z;
  }
  
  inline float wrap2pi(float z)
  {
    while (z <= -M_PI) z += 2*M_PI;
    while (z > M_PI) z -= 2*M_PI;
    
    return z;
  }

}
#endif

