/*
 * environment.hh (C) Inaki Rano 2022
 * 
 * Definition of environment class for the 2D multi-robot simulator
 * and the functions related to the environment: distance(), angle()
 * and wrap()
 *
 */
#include "base.hh"
#include "environment.hh"

namespace mrs {

  // Pointer to the environment initilised to null until an
  // environment object is created
  Environment *Environment::m_envp = NULL;

  // Function to compute distances returns Euclidean distance if there
  // is no environment object created
  float distance(const Position2d & p0, const Position2d & p1)
  {
    if ((Environment::m_envp == NULL) || Environment::m_envp->m_isEuclidean)
      return (p0-p1).norm();
    
    return Environment::m_envp->dist(p0, p1);
  }

  // Function to compute angle, returns normal angle if there is no
  // environment object created
  float angle(const Position2d & p0, const Position2d & p1)
  {
    if ((Environment::m_envp == NULL) || Environment::m_envp->m_isEuclidean) {
      Position2d diff(p1 - p0);
      return atan2(diff[1], diff[0]);
    }
    
    return Environment::m_envp->ang(p0, p1);
  }
  // Wrap the point to fall again in the environment
  Position2d wrap(const Position2d & p)
  {
    if ((Environment::m_envp == NULL) || Environment::m_envp->m_isEuclidean)
      return p;
    
    return Environment::m_envp->wrap(p);
  }

  // Function to calculate average of a vector of points
  Position2d average(const std::vector<Position2d> & pts)
  {
    // Calculate standard average
    Position2d mP(Position2d::Zero());
    for (unsigned int ii = 0; ii < pts.size(); ii++)
      mP += pts[ii];
    mP /= (float)pts.size();
    
    if ((Environment::m_envp == NULL) || Environment::m_envp->m_isEuclidean)
      return mP;

    // Search for the true average in a spherical topology. This is an
    // adaptation of what I understood from
    //
    // https://www.youtube.com/watch?v=cYVmcaRAbJg
    //
    // For angular an variables and a discrete set of angles (a1,
    // a2,...,aN) the real average on the circle can be selected among
    // the following mean angle candidates:
    //
    // am_k = 1/N * \Sum_i(ai) + k*2*pi/N
    //
    // for k=1,2,...,N. The actual mean will be the one minimising the
    // sum of the distances to the original angles, i.e. fulfilling
    // the equation of the mean in a manyfold (M):
    //
    // am = argmin_{a\in M}\Sum_i d(a_i,a)
    int sz(pts.size());
    std::vector<Position2d> mPcandidates;
    Position2d mPc;
    for (unsigned int ii = 0; ii < sz; ii++) {
      mPc[0] = wrapVar(mP[0] + ii * Environment::m_envp->m_size[0] / sz,
		       Environment::m_envp->m_size[0]);
      for (unsigned int jj = 0; jj < sz; jj++) {
	mPc[1] = wrapVar(mP[1] + jj * Environment::m_envp->m_size[1] / sz,
			 Environment::m_envp->m_size[1]);
	mPcandidates.push_back(mPc);
      }
    }
    float minDist((float)sz * Environment::m_envp->m_size.norm());
    for (unsigned int ii = 0; ii < mPcandidates.size(); ii++)
      {
	float currDist(0);
	for (unsigned int jj = 0; jj < pts.size(); jj++)
	  currDist += distance(pts[jj], mPcandidates[ii]);
	if (currDist < minDist) {
	  minDist = currDist;
	  mP = mPcandidates[ii];
	}
      }
    
    return mP;
  }

  // Function to calculate average of a vector of angles
  float averageAngle(const std::vector<float> & angs)
  {
    // Calculate standard average
    float mAng(0);
    for (unsigned int ii = 0; ii < angs.size(); ii++)
      mAng += angs[ii];
    mAng /= (float)angs.size();
    
    // Search for the true average in a circle. This is an adaptation
    // of what I understood from:
    //
    // https://www.youtube.com/watch?v=cYVmcaRAbJg
    //
    // For angular an variables and a discrete set of angles (a1,
    // a2,...,aN) the real average on the circle can be selected among
    // the following mean angle candidates:
    //
    // am_k = 1/N * \Sum_i(ai) + k*2*pi/N
    //
    // for k=1,2,...,N. The actual mean will be the one minimising the
    // sum of the distances to the original angles, i.e. fulfilling
    // the equation of the mean in a manyfold (M):
    //
    // am = argmin_{a\in M}\Sum_i d(a_i,a)
    int sz(angs.size());
    float minDist((float)sz * 2 * M_PI);
    float mAngCandidate(0), currMean(mAng);
    for (unsigned int ii = 0; ii < sz; ii++) {
      float currDist(0);
      mAngCandidate = wrap2pi(mAng + ii * 2 * M_PI /(float)sz);
      for (unsigned int jj = 0; jj < angs.size(); jj++)
	currDist += fabs(wrap2pi(angs[jj] - mAngCandidate));
      if (currDist < minDist) {
	minDist = currDist;
	currMean = mAngCandidate;
      }
    }
    return currMean;
  }

  // Function to calculate average of a vector of points using a grid
  // search (gradient descent might lead to faster results but it
  // might also get trapped in local minima, as the sum of the
  // distances might be non-convex)
  float averageAngleW(const std::vector<float> & angs,
		      const std::vector<float> & w)
  {
    if (angs.size() != w.size())
      {
	std::cerr << "averageAngleW(): Weight error!" << std::endl;
      }
    // Copy vector of angles
    Eigen::VectorXf A(angs.size());
    for (unsigned int ii = 0; ii < A.size(); ii++)
      A[ii] = angs[ii];
    // Copy vector of weights
    Eigen::VectorXf W(w.size());
    for (unsigned int ii = 0; ii < A.size(); ii++)
      {
	W[ii] = w[ii];
	if (W[ii] < 0)
	  std::cerr << "averageAngleW(): Warning negative weight!"
		    << std::endl;
      }
	  
    // Number of points to evaluate the distortion
    int sz(10 * angs.size());

    // Mean weighted angle and previous mean weighted angle
    float mAngPrev(0), mAng(0);
    // Search range width
    float delta(2*M_PI);
    // Search range
    float range[2] = {-M_PI, M_PI};

    Eigen::VectorXf mAngCandidates(sz);
    Eigen::VectorXf distortion(sz);
    bool meanImproves(true);
    float tol(1e-6);
    while (meanImproves) {
      
      // Select candidate angles
      for (unsigned int ii = 0; ii < sz; ii++)
	mAngCandidates[ii] = wrap2pi(range[0] + ii * delta / sz +
				       delta / (2 * sz));
      
      // Calculate distortion for the candidate means
      for (unsigned int ii = 0; ii < sz; ii++)
	{
	  Eigen::VectorXf aux(A - mAngCandidates[ii] *
			      Eigen::VectorXf::Ones(A.size()));
	  for (unsigned int jj = 0; jj < aux.size(); jj++)
	    aux[jj] = wrap2pi(aux[jj]);
	  aux = aux.array().pow(2);
	  distortion[ii] = W.transpose() * aux;
	}      
      
      // Find the point of min distotion and new weighted average
      mAngPrev = mAng;
      unsigned int idx(0);
      mAng = mAngCandidates[idx];
      for (unsigned int ii = 1; ii < sz; ii++)
	if (distortion[ii] < distortion[idx]) {
	  idx = ii;
	  mAng = mAngCandidates[idx];
	}
      
      meanImproves = (fabs(wrap2pi(mAng - mAngPrev)) >= tol);
      delta /= 2.0;
      range[0] = wrap2pi(mAng - delta / 2);
      range[1] = wrap2pi(mAng + delta / 2);
    }
    
    return mAng;
  }
  
  
  // Constructor sets the limit points, size and pointer to the
  // environment. Checks validity of the arguments and that there are
  // no more than one environment object
  Environment::Environment(const Position2d & min, const Position2d & max,
			   bool isEuclidean)
  {
    assert(m_envp == NULL);
    assert(min.size() == max.size());
    for (unsigned int ii = 0; ii < min.size(); ii++)
      assert(min[ii] < max[ii]);
    m_min = min;
    m_max = max;
    m_size = max - min;
    m_isEuclidean = isEuclidean;
    m_envp = this;
  }

  
  bool
  Environment::isFree(const Position2d & x, double r,
		      const SwarmPtr swarm) const
  {
    // If no swarm is passed as an argument in only checks that the
    // point is within the limits of the environment
    if (swarm == nullptr) {
      if ((x[0] > m_min[0]) && (x[0] <= m_max[0]) &&
	  (x[1] > m_min[1]) && (x[1] <= m_max[1]))
	return true;
      return false;
    }
    else
      for (std::vector<RobotPtr>::const_iterator iit = swarm->begin();
	     iit != swarm->end(); ++iit)
	  {
	    double lim(r + (*iit)->settings().radius);
	    if (dist(x, (*iit)->position()) < lim)
	      return false;
	  }
    
    return true;
  }

  Position2d
  Environment::random() const
  {
    return Position2d(0.5 * (m_min + m_max) +
		      Position2d::Random().cwiseProduct(0.5 * m_size));
  }

  Position2d
  Environment::random(float r, const SwarmPtr swarm, int & trials) const
  {
    Position2d p;
    bool isPFree(true);
    do {
      p = random();
      isPFree = isFree(p, r, swarm);
    } while ((trials-- > 0) &&(!isPFree));

    return p;
  }
  
  float
  Environment::dist(const Position2d & p0, const Position2d & p1) const
  {
    Position2d diff(p0 - p1);
    for (unsigned int ii = 0; ii < diff.size(); ii++)
      diff[ii] = wrapVar(diff[ii], m_size[ii]);

    return diff.norm();
  }

  float
  Environment::ang(const Position2d & p1, const Position2d & p0) const
  {
    Position2d diff(p0 - p1);
    for (unsigned int ii = 0; ii < diff.size(); ii++)
      diff[ii] = wrapVar(diff[ii], m_size[ii]);

    return atan2(diff[1], diff[0]);;
  }

  Position2d
  Environment::wrap(const Position2d & p) const
  {
    Position2d wp(p);
    
    for (unsigned int ii = 0; ii < p.size(); ii++)
      wp[ii] = wrapVar(p[ii], m_size[ii]);

    return wp;
  }
  
  std::ostream & operator<< (std::ostream & ofd, const mrs::Environment & env)
  {
    ofd << "Environment: " << "minP: " << env.min().transpose() << std::endl;
    ofd << "Environment: " << "maxP: " << env.max().transpose() << std::endl;
	 
    return ofd;
  }
  

}
