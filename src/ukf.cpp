#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  // initial state vector
  n_x_=5;
  x_ = VectorXd(n_x_);
  x_ << 0.,0.,5.,0.,0.;
  
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  P_(0,0)=1.;
  P_(1,1)=1.;
  P_(2,2)=1.;
  P_(3,3)=1.;
  P_(4,4)=0.1;
  
  
  n_aug_=n_x_+2;
  
  n_sigma=1+2*n_aug_;
  
  lambda_ = 3 - n_x_;
  
  weights_=Eigen::VectorXd(n_sigma);
  weights_(0)=lambda_/(lambda_+n_aug_);
  for (int i=1;i<n_sigma;i++) {
    weights_(i)=0.5/(lambda_+n_aug_);
  }
  
  Xsig_aug_= Eigen::MatrixXd(n_aug_,n_sigma);
  Xsig_aug_.fill(0.0);
  
  Xsig_pred_ = MatrixXd(n_x_, n_sigma);
  Xsig_pred_.fill(0.0);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
  
  R_laser = MatrixXd(2,2);
  R_laser << std_laspx_*std_laspx_,0,
  0,std_laspy_*std_laspy_;
  
  
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  R_radar = MatrixXd(n_radar,n_radar);
  R_radar <<std_radr_*std_radr_,0,0,
  0,std_radphi_*std_radphi_,0,
  0,0,std_radrd_*std_radrd_;
  
  is_initialized_=false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    Initialize(meas_package);
    return;
  }
  
  double dt=(meas_package.timestamp_-time_us_)/1000000.;
  time_us_=meas_package.timestamp_;
  Prediction(dt);
  
  if ((meas_package.sensor_type_==MeasurementPackage::RADAR) && use_radar_) {
    UpdateRadar(meas_package);
  }
  
  if ((meas_package.sensor_type_==MeasurementPackage::LASER) && use_laser_) {
    UpdateLidar(meas_package);
  }
  
}

void UKF::Initialize(MeasurementPackage meas_package) {
  // first measurement
  if ((meas_package.sensor_type_==MeasurementPackage::RADAR) && use_radar_) {
    /**
     Convert radar from polar to cartesian coordinates and initialize state.
     */
    float r=meas_package.raw_measurements_[0];
    float phi=meas_package.raw_measurements_[1];
    
    x_[0]=r*cos(phi);
    x_[1]=r*sin(phi);
    time_us_=meas_package.timestamp_;
    
    // set realistic values for the covariance matrix
    P_(0,0)=std_radr_*std_radr_;
    P_(1,1)=std_radr_*std_radr_;
    is_initialized_ = true;
    
  }
  else if ((meas_package.sensor_type_ == MeasurementPackage::LASER)&&use_laser_) {
    /**
     Initialize state.
     */
    x_[0]=meas_package.raw_measurements_[0];
    x_[1]=meas_package.raw_measurements_[1];
    time_us_=meas_package.timestamp_;
    
    // set realistic values for the covariance matrix
    P_(0,0)=std_laspx_*std_laspx_;
    P_(1,1)=std_laspy_*std_laspx_;
    
    is_initialized_ = true;
    
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  
  cout<<"\nBefore prediction\n" << x_<<std::endl;
  Eigen::MatrixXd Xsig_aug;
  
  GenerateAugmentedSigmaPoints();
  SigmaPointPrediction(dt);
  
  //predict state mean
  x_=(Xsig_pred_*weights_).rowwise().sum();
  
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sigma; i++) {  //iterate over sigma points
                                       // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3)=tools.RangeLimitPhi(x_diff(3));
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
  cout<<"\nAfter prediction\n" << x_<<std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  VectorXd z=meas_package.raw_measurements_;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_laser, n_sigma);
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_laser);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_laser,n_laser);
  
  Zsig.fill(0.0);
  //transform sigma points into measurement space
  for (int i=0;i<Zsig.cols();i++) {
    float px=Xsig_pred_(0,i);
    float py=Xsig_pred_(1,i);
    //float v=Xsig_pred_(2,i);
    //float psi=Xsig_pred_(3,i);
    //float psi_dot=Xsig_pred_(4,i);
    
    Zsig(0,i)=px;
    Zsig(1,i)=py;
  }
  //calculate mean predicted measurement
  z_pred=(Zsig*weights_).rowwise().sum();
  
  
  //calculate measurement covariance matrix S
  MatrixXd Zd=Zsig.colwise()-z_pred;
  S.fill(0.0);
  for (int i=0;i<Zsig.cols();i++) {
    S = S + weights_(i)*Zd.col(i)*Zd.col(i).transpose();
  }
  S=S+R_laser;
  
  
  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;
  
  
  cout <<"\n\nUpdateStateFromLaser\nmeasurement\n " <<z <<endl;
  cout <<"\nprediction \n"<< z_pred<<endl;
  cout <<"\ndelta \n"<< (z-z_pred)<<endl;
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_laser);
  
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0;i<weights_.size();i++) {
    VectorXd xd=Xsig_pred_.col(i)-x_;
    xd(3)=tools.RangeLimitPhi(xd(3));
    
    VectorXd zd=Zsig.col(i)-z_pred;
    
    Tc=Tc+weights_(i)*xd*zd.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K=Tc*S.inverse();
  //update state mean and covariance matrix
  x_=x_+K*(z-z_pred);
  P_=P_-K*(S*K.transpose());
  cout<<"\nAfter Update Laser\n" << x_<<std::endl;
  /**
   TODO:
   
   You'll also need to calculate the radar NIS.
   */
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
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_radar, n_sigma);
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_radar);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_radar,n_radar);
  
  Zsig.fill(0.0);
  //transform sigma points into measurement space
  for (int i=0;i<Zsig.cols();i++) {
    float px=Xsig_pred_(0,i);
    float py=Xsig_pred_(1,i);
    float v=Xsig_pred_(2,i);
    float psi=Xsig_pred_(3,i);
    //float psi_dot=Xsig_pred_(4,i);
    
    Zsig(0,i)=std::sqrt(px*px+py*py);
    Zsig(1,i)=std::atan2(py,px);
    if (std::abs(Zsig(0,i))>0.0001) {
      Zsig(2,i)=(px*cos(psi)*v+py*sin(psi)*v)/Zsig(0,i);
    }
  }
  //calculate mean predicted measurement
  z_pred=(Zsig*weights_).rowwise().sum();
  
  
  //calculate measurement covariance matrix S
  MatrixXd Zd=Zsig.colwise()-z_pred;
  for (int i=0;i<Zd.cols();i++) {
    Zd(1,i)=tools.RangeLimitPhi((Zd(1,i)));
  }
  S.fill(0.0);
  for (int i=0;i<Zsig.cols();i++) {
    S = S + weights_(i)*Zd.col(i)*Zd.col(i).transpose();
  }
  S=S+R_radar;
  
  
  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;
  
  
  cout <<"\n\nUpdateStateFromRadar\nmeasurement\n " <<z <<endl;
  cout <<"\nprediction \n"<< z_pred<<endl;
  cout <<"\ndelta \n"<< (z-z_pred)<<endl;
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_radar);
  
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0;i<weights_.size();i++) {
    VectorXd xd=Xsig_pred_.col(i)-x_;
    xd(3)=tools.RangeLimitPhi(xd(3));
    
    VectorXd zd=Zsig.col(i)-z_pred;
    zd(1)=tools.RangeLimitPhi(zd(1));
    
    Tc=Tc+weights_(i)*xd*zd.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K=Tc*S.inverse();
  //update state mean and covariance matrix
  x_=x_+K*(z-z_pred);
  P_=P_-K*(S*K.transpose());
  cout<<"\nAfter Update Radar\n" << x_<<std::endl;
  /**
   TODO:
   
   You'll also need to calculate the radar NIS.
   */
};


/**
 * Generate the augmented sigma points, based on x_, P_, std_a_, and std_yawdd.
 * Output goes into X_sig_aug_
 */

void UKF::GenerateAugmentedSigmaPoints() {
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  
  //create augmented mean state
  x_aug.head(5)=x_;
  x_aug[5]=0;
  x_aug[6]=0;
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;
  //create square root matrix
  cout <<"P_aug\n"<<P_aug<<std::endl;
  MatrixXd L=P_aug.llt().matrixL();
  //create augmented sigma points
  
  int i=0;
  Xsig_aug_.col(i++)=x_aug;
  for (int j=0;j<n_aug_;j++) {
    Xsig_aug_.col(i++)=x_aug+std::sqrt(lambda_+n_aug_) *L.col(j);
  }
  for (int j=0;j<n_aug_;j++) {
    Xsig_aug_.col(i++)=x_aug-std::sqrt(lambda_+n_aug_) * L.col(j);
  }
  cout <<"Xsig_aug_\n"<<Xsig_aug_<<endl;
  //TODO: check whether this is needed. Probably not, due to spread calculation later
  //  for (int j=0;j<n_sigma;j++) {
  //    Xsig_aug_(3,j)=tools.RangeLimitPhi(Xsig_aug_(3,j));
  //  }
  
  //print result
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  
}

/**
 * Predict the sigmapoints. Input is Xsig_aug_. It implements the process model.
 * Output goes to Xsig_pred_.
 */
void UKF::SigmaPointPrediction(double dt) {
  
  for (int i=0;i<n_sigma;i++) {
    float px=Xsig_aug_(0,i);
    float py=Xsig_aug_(1,i);
    float v=Xsig_aug_(2,i);
    float psi=Xsig_aug_(3,i);
    float psi_dot=Xsig_aug_(4,i);
    float nu_a=Xsig_aug_(5,i);
    float nu_psi_dot=Xsig_aug_(6,i);
    
    if (std::abs(psi_dot)<0.001) {
      Xsig_pred_(0,i)=px+v*cos(psi)*dt + 0.5 * dt * dt * cos(psi)*nu_a;
      Xsig_pred_(1,i)=py+v*sin(psi)*dt + 0.5 * dt * dt * sin(psi)*nu_a;
      
    } else {
      double d=sin(psi+psi_dot*dt)-sin(psi);
      d=v/psi_dot*d;
      d=d+0.5*dt*dt*cos(psi)*nu_a;
      d=px+d;
      Xsig_pred_(0,i)=px+v/psi_dot*(sin(psi +psi_dot*dt)-sin(psi)) + 0.5 * dt * dt * std::cos(psi)*nu_a;
      Xsig_pred_(1,i)=py+v/psi_dot*(-cos(psi+psi_dot*dt)+cos(psi)) + 0.5 * dt * dt * sin(psi)*nu_a;
      
    }
    Xsig_pred_(2,i)=v+dt*nu_a;
    //TODO: probably also not needed here to range limit
    Xsig_pred_(3,i)=psi+psi_dot*dt + 0.5*dt*dt*nu_psi_dot;
    Xsig_pred_(4,i)=psi_dot+dt*nu_psi_dot;
  }
  
  //print result
  std::cout << "Xsig_pred\n" << Xsig_pred_ << std::endl;
  
}



