#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here..
  */
 VectorXd rmse=VectorXd(4);
 rmse<<0,0,0,0;

 if(estimations.size()!=ground_truth.size() || estimations.size()==0){
    cout<<"Check size of estimations. Estimation and Ground Truth Vector doesn't match in size";
    return rmse;
 }
 for(unsigned int i=0; i<estimations.size(); i++){
    Eigen::VectorXd residual= estimations[i]- ground_truth[i];
    residual=residual.array()*residual.array();
    rmse+=residual;
  }
	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}