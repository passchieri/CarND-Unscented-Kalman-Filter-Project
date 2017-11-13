//
//  state.cpp
//  UnscentedKF
//
//  Created by Igor Passchier on 13/11/2017.
//
#include "ukf.h"
#include "state.h"
#include <iostream>
#include <fstream>

State::State(void) {
  
  nisFileLaser.open("laser_data.csv");
  nisFileRadar.open("radar_data.csv");
  dataFile.open("data.csv" );
}
