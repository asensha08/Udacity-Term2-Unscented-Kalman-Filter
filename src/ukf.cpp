#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 1.18;  

  //std_a_ = 2;//p=50/0.4256//0.4391...p=20//0.4099//0.4380//
  //std_a_ = 2.4;//p=20//0.4022,0.1029//0.4297
  //std_a_ = 2.2;//p=20//0.4001,0.1006//0.4292
  //std_a_ = 2.15;//p=20//0.3988,0.1004//0.4282
  //std_a_ = 2.08;//p=20//0.3973,0.1002//0.4272
  //std_a_ = 2.02;//p=20//0.3962,0.1001//0.4265

  //std_a_ = 1.95;//p=24//0.3962,0.1001//0.4265

  //Tuned Value- Amazing Results
  std_a_ = 0.7;//p=24//0.3962,0.1001//0.4265
  
  // Process nose standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 0.5;

  //std_yawdd_ = 0.3;//0.4256//0.4391.p=20//0.4099//0.4380//
  //std_yawdd_ = 0.6;//p=20//0.4022,0.1029//0.4297
  //std_yawdd_ = 0.8;//p=20//0.4001,0.1006//0.4292
  //std_yawdd_ = 0.75;//p=20//0.3988,0.1004//0.4282
  //std_yawdd_ = 0.70;//p=20//0.3973,0.1002//0.4272
  //std_yawdd_ = 0.65;//p=20//0.3962,0.1001//0.4265

  //Tuned Value - Amazing Results
  std_yawdd_ = 0.65;//p=20//0.3962,0.1001//0.4265
  
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_=false;

  n_x_=5;

  n_aug_=7;

  lambda_=3-n_x_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  time_us_=0;

  weights_=VectorXd(2*n_aug_ +1);

  NIS_Lidar_=0;

  NIS_radar_=0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  //if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
      //(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)) {
        if(is_initialized_==false){
        

        


     // init timestamp
        time_us_ = meas_package.timestamp_;
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
        if(meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_){

   /**
    * Initialize state.
    * Note: I have initialized the state and covriance depending on sensor type 
    * 
  
  */
            x_(0) = meas_package.raw_measurements_(0);
            x_(1) = meas_package.raw_measurements_(1);
            if (fabs(x_(0)) < 0.0001 and fabs(x_(1)) < 0.0001){
              x_(0) = 0.0001;
              x_(1) = 0.0001;
              }
              x_<<meas_package.raw_measurements_[0],
                  meas_package.raw_measurements_[1],
                  0.1,
                  0.1,
                  0;
              P_<<0.08, 0, 0, 0, 0,
            0, 0.08, 0, 0, 0,
            0, 0, 24, 0, 0,
            0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0.01;
              
                 
              //cout<<"Laser Measurement"<<endl;
              //cout<<x_(0)<<'\n'<<x_(1);
              }
        else if(meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_){
          float rho= meas_package.raw_measurements_[0];
          float phi= meas_package.raw_measurements_[1];
          float rho_dot= meas_package.raw_measurements_[2];

          float px= rho*cos(phi);
          float py= rho*sin(phi);
   //float v= sqrt((rho_dot*cos(phi)*rho_dot*cos(phi)) +(rho_dot*sin(phi)*rho_dot*sin(phi)));
   //float yaw_m= atan2(py,px);
          //x_(0)=px;
          //x_(1)=py;
          x_<<    px,
                  py,
                  meas_package.raw_measurements_[2],
                  0,
                  0;
          P_<<0.1, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0,
            0, 0, 20, 0, 0,
            0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0.01;
   //x_(2)=v;
   //x_(3)=yaw_m;

          if (fabs(x_(0)) < 0.0001 and fabs(x_(1)) < 0.0001)
          {
              x_(0) = 0.0001;
              x_(1) = 0.0001;
          }
        
        }
 is_initialized_=true;
 return;
        }
 float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
 time_us_ = meas_package.timestamp_;

    Prediction(dt);

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    }
      //}

    // print the output
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
  //cout << "x_ = " << NIS_Lidar_ << endl;
  //cout << "P_ = " << NIS_radar_ << endl;
}



/** 
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
 MatrixXd Xsig=MatrixXd(n_x_, 2*n_x_+1);
 lambda_ = 3 - n_x_;

 MatrixXd A=P_.llt().matrixL();

 Xsig.col(0)=x_;

 for (int i=0; i<n_x_; i++){
   Xsig.col(i+1)=x_ + sqrt(lambda_+n_x_)*A.col(i);
   Xsig.col(i+1+n_x_)= x_ - sqrt(lambda_+n_x_)*A.col(i);

 }

 //Augementing the Sigma Points

 VectorXd X_aug= VectorXd(n_aug_);

 X_aug.head(5)=x_;
 X_aug(5)=0;
 X_aug(6)=0;

 //Augemented Covariance Matrix

 MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

 P_aug.fill(0.0);

 P_aug.topLeftCorner(5,5)=P_;
 P_aug(5,5) = std_a_*std_a_;
 P_aug(6,6) = std_yawdd_*std_yawdd_;

 //Square root Covariance Matrix

 MatrixXd L=P_aug.llt().matrixL();

 MatrixXd Xsig_aug=MatrixXd(n_aug_, 2*n_aug_ +1);

 Xsig_aug.col(0)= X_aug;

 lambda_=3-n_aug_;

 for (int i=0; i<n_aug_; i++){
   Xsig_aug.col(i+1)=X_aug + sqrt(lambda_+n_aug_)* L.col(i);
   Xsig_aug.col(i+1+n_aug_)= X_aug - sqrt(lambda_+n_aug_) *L.col(i);

 }


  /*****************************************************************************
  *  Predict Sigma Points
  ****************************************************************************/
//predict sigma points

for(int i=0; i<2*n_aug_+1; i++){

  double px=Xsig_aug(0,i);
  double py=Xsig_aug(1,i);
  double v=Xsig_aug(2,i);
  double yaw=Xsig_aug(3,i);
  double yawd=Xsig_aug(4,i);
  double nu_a=Xsig_aug(5,i);
  double nu_yawdd=Xsig_aug(6,i);

 //predicted state values
  double px_p, py_p;

    //avoid division by zero
  if (fabs(yawd) > 0.001) {
        px_p = px + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = py + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
  }
  else {
        px_p = px + v*delta_t*cos(yaw);
        py_p = py + v*delta_t*sin(yaw);
  }

  double v_p = v;
  double yaw_p = yaw + yawd*delta_t;
  double yawd_p = yawd;

    //add noise
  px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
  py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
  v_p = v_p + nu_a*delta_t;

  yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
  yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
  Xsig_pred_(0,i) = px_p;
  Xsig_pred_(1,i) = py_p;
  Xsig_pred_(2,i) = v_p;
  Xsig_pred_(3,i) = yaw_p;
  Xsig_pred_(4,i) = yawd_p;
}

//Mean and Covariance Prediction

//Weights Initializtion

double weights = lambda_/(lambda_+n_aug_);
weights_(0)=weights;

for(int i=1; i<2*n_aug_+1; i++){
  double weight_i=0.5/(lambda_+n_aug_);
  weights_(i)= weight_i;
}

//VectorXd x=VectorXd(n_x_);
x_.fill(0.0);

//MatrixXd P=MatrixXd(n_x_,n_x_);


for(int i=0; i<2*n_aug_ +1; i++){
  x_=x_+ (weights_(i)*Xsig_pred_.col(i));
} 

VectorXd x_diff=VectorXd(n_x_);
P_.fill(0.0);
for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
  x_diff = Xsig_pred_.col(i) - x_;
  //angle normalization
  if(x_diff(3) < -M_PI){
    x_diff(3) += 2 * M_PI;
  }else if (x_diff(3) > M_PI){
    x_diff(3) -= 2 * M_PI;
  }
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
}
} 

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  VectorXd z=meas_package.raw_measurements_;

  int n_z=2;

  MatrixXd Zsig=MatrixXd(n_z,((2*n_aug_)+1));

  for(int i=0; i<2*n_aug_ +1; i++){
    double p_x=Xsig_pred_(0,i);
    double p_y=Xsig_pred_(1,i);


    Zsig(0,i)=p_x;
    Zsig(1,i)=p_y;
  }

  //Measurement Precdiction: Mean and Covariance

  VectorXd z_pred= VectorXd(n_z);

  z_pred.fill(0.0);

  for (int i=0; i<2*n_aug_ +1; i++){
    z_pred=z_pred+ (weights_(i)*Zsig.col(i));
  }

  MatrixXd S= MatrixXd(n_z,n_z);
  S.fill(0.0);

  VectorXd z_diff= VectorXd(n_z);
  for(int i=0; i<2 * n_aug_ +1; i++){

    z_diff = Zsig.col(i)-z_pred;

    S=S+ weights_(i) *z_diff * z_diff.transpose();
  }

  MatrixXd R= MatrixXd(n_z,n_z);

  R<<std_laspx_*std_laspx_,0,
     0,std_laspy_*std_laspy_;

  S=S+R;

  //Measurement Update

  MatrixXd Tc= MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  VectorXd x_diff= VectorXd(n_x_);

  for(unsigned int i=0; i<2 * n_aug_ + 1; i++){
    x_diff=Xsig_pred_.col(i)- x_;
    z_diff=Zsig.col(i)- z_pred;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  MatrixXd K = Tc * S.inverse();

  //residual
  z_diff = z - z_pred;

  //calculate NIS
   NIS_Lidar_= z_diff.transpose() * S.inverse() * z_diff;

  //update mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  VectorXd z=meas_package.raw_measurements_;

  int n_z=3 ;

  MatrixXd Zsig=MatrixXd(n_z,2*n_aug_+1);

  for(int i=0; i<2*n_aug_ +1; i++){
    double p_x=Xsig_pred_(0,i);
    double p_y=Xsig_pred_(1,i);

    double v_radar=Xsig_pred_(2,i);

    double yaw_radar=Xsig_pred_(3,i);

    double vx= v_radar*cos(yaw_radar);
    double vy= v_radar*sin(yaw_radar);

    Zsig(0,i)=sqrt(p_x*p_x + p_y*p_y);
    Zsig(1,i)=atan2(p_y,p_x);
    Zsig(2,i)=(p_x*vx + p_y*vy) / sqrt(p_x*p_x + p_y*p_y);
  }

  //Measurement Precdiction: Mean and Covariance

  VectorXd z_pred= VectorXd(n_z);

  z_pred.fill(0.0);

  for (unsigned int i=0; i<2*n_aug_ +1; i++){
    z_pred=z_pred+ weights_(i)*Zsig.col(i);
  }

  MatrixXd S= MatrixXd(n_z,n_z);
  S.fill(0.0);

  VectorXd z_diff= VectorXd(n_z);

  for(int i=0; i<2 * n_aug_ +1; i++){

    z_diff = Zsig.col(i)-z_pred;

     //angle normalization

    if(z_diff(1) < -M_PI){
      z_diff(1) += 2 * M_PI;
    }
    else if (z_diff(1) > M_PI){
    z_diff(1) -= 2 * M_PI;
    }

    S=S+ weights_(i) *z_diff * z_diff.transpose();
  }

  MatrixXd R= MatrixXd(n_z,n_z);

  R<<std_radr_*std_radr_,0,0,
     0,std_radphi_*std_radphi_,0,
     0,0,std_radrd_*std_radrd_;

  S=S+R;

  //Measurement Update

  MatrixXd Tc= MatrixXd(n_x_, n_z);

  Tc.fill(0.0);

  VectorXd x_diff= VectorXd(n_x_);

  for(unsigned int i=0; i<2*n_aug_+1; i++){
    x_diff=Xsig_pred_.col(i)- x_;
    z_diff=Zsig.col(i)- z_pred;

     //residual
   z_diff = z - z_pred;

   //angle normalization
   if(z_diff(1) < -M_PI){
      z_diff(1) += 2 * M_PI;
    }
    else if (z_diff(1) > M_PI){
    z_diff(1) -= 2 * M_PI;
    }

     //residual

   //angle normalization
   if(x_diff(3) < -M_PI){
      x_diff(3) += 2 * M_PI;
    }
    else if (x_diff(3) > M_PI){
    x_diff(3) -= 2 * M_PI;
    }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  MatrixXd K = Tc * S.inverse();

  //residual
   z_diff = z - z_pred;

   //angle normalization
   if(z_diff(1) < -M_PI){
      z_diff(1) += 2 * M_PI;
    }
    else if (z_diff(1) > M_PI){
    z_diff(1) -= 2 * M_PI;
    }
  

  //calculate NIS
   NIS_radar_= z_diff.transpose() * S.inverse() * z_diff;

  //update mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

}
