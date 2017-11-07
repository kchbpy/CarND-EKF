#include <iostream>
#include "tools.h"
// #include <math.h>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if(estimations.size() != ground_truth.size()
    || estimations.size() == 0 || ground_truth.size() == 0){
    return rmse;
  }

  for(int i = 0;i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj = MatrixXd(3,4);
  float p_x = x_state[0];
  float p_y = x_state[1];
  float v_x = x_state[2];
  float v_y = x_state[3];

  if(p_x == 0 && p_y == 0)
  {
    Hj << 0,0,0,0,
          0,0,0,0,
          0,0,0,0;
  }
  else 
  {
    Hj << p_x/pow(p_x*p_x+p_y*p_y,0.5), p_x/pow(p_x*p_x+p_y*p_y,0.5),0,0,
          -p_y/(p_x*p_x+p_y*p_y)     , p_x/(p_x*p_x+p_y*p_y),        0,0,
          p_y*(v_x*p_y-v_y*p_x)/pow(p_x*p_x+p_y*p_y,3.0/2),
          p_x*(v_y*p_x-v_x*p_y)/pow(p_x*p_x+p_y*p_y,3.0/2),
          p_x/pow(p_x*p_x+p_y*p_y,0.5),
          p_y/pow(p_x*p_x+p_y*p_y,0.5);
  }
  return Hj;

}
