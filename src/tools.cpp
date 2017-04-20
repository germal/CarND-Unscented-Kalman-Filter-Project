/*______________________________________________________________________________
                                                                           80->|
  main.cpp
  This module implements the Tools class
*/

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Code from my answer to L5.22 quiz
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity of the inputs:
  // the estimation vector size should not be zero
  assert(estimations.size() > 0);
  // the estimation vector size should equal ground truth vector size
  assert(estimations.size() == ground_truth.size());

  //accumulate squared residuals
  for(int i = 0; i < estimations.size(); i++) {
    VectorXd r = estimations[i] - ground_truth[i];
    r = r.array() * r.array();
    rmse += r;
  }

  //calculate the mean
  rmse = rmse / double(estimations.size());

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}
