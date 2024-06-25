#include "SmoothingFilter.h"

#include <cmath>
#include <numeric>
#include <iostream>

// for dq
bool SmoothingFilter::applyMedianFilter(cnoid::BodyPtr& genRobot) {
  for(int i=0;i<genRobot->numJoints();i++){
    dqFilterWindow[i].push_back(genRobot->joint(i)->dq());
        
    if (dqFilterWindow[i].size() > windowSize) dqFilterWindow[i].erase(dqFilterWindow[i].begin());
    
    std::vector<double> sorted_data = dqFilterWindow[i];
    std::sort(sorted_data.begin(), sorted_data.end());

    size_t size = sorted_data.size();
    size_t median_index = size / 2;
    if (size % 2 == 0) {
      genRobot->joint(i)->dq() = (sorted_data[median_index - 1] + sorted_data[median_index]) / 2.0;
    } else {
      genRobot->joint(i)->dq() = sorted_data[median_index];
    }
  }

  return true;
}

// for q
bool SmoothingFilter::applyAverageFilter(cnoid::BodyPtr& genRobot) {
  for(int i=0;i<genRobot->numJoints();i++){
    qFilterWindow[i].push_back(genRobot->joint(i)->q());

    if (qFilterWindow[i].size() > windowSize) qFilterWindow[i].erase(qFilterWindow[i].begin());
    
    double sum = std::accumulate(qFilterWindow[i].begin(), qFilterWindow[i].end(), 0.0);   
    genRobot->joint(i)->q() = sum / qFilterWindow[i].size();
    
  }
     
  return true;
}
