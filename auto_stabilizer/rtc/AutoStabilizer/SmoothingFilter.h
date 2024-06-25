#ifndef SMOOTHINGFILTER_H
#define SMOOTHINGFILTER_H

#include <vector>
#include <cnoid/Body>

class SmoothingFilter{
public:
  double windowSize = 3;
  std::vector<std::vector<double> > qFilterWindow;
  std::vector<std::vector<double> > dqFilterWindow;
  
  void init(const cnoid::BodyPtr& genRobot) {
    qFilterWindow = std::vector<std::vector<double> > (genRobot->numJoints());
    dqFilterWindow = std::vector<std::vector<double> > (genRobot->numJoints());
  };

  bool applyMedianFilter(cnoid::BodyPtr& genRobot);   // for dq
  bool applyAverageFilter(cnoid::BodyPtr& genRobot);  // for q
  /* bool applyMedianFilter(cnoid::BodyPtr& genRobot, std::vector<std::vector<double>>& filterWindow);   // for dq */
  /* bool applyLowpassFilter(cnoid::BodyPtr& genRobot, std::vector<std::vector<double>>& filterWindow);  // for q */
    
};

#endif
