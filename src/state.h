//
//  state.h
//  UnscentedKF
//
//  Created by Igor Passchier on 13/11/2017.
//

#ifndef state_h
#define state_h

#include <stdio.h>
#include "ukf.h"
class State {
public:
  ofstream nisFileLaser;
  ofstream nisFileRadar;
  ofstream dataFile;
  UKF ukf;
  VectorXd RMSE;

  
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  
  
  State(void);
};


#endif /* state_h */
