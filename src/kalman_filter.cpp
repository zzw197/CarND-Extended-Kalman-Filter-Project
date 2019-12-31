#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //Estimation
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y_ = z - H_ * x_;
  UpdateY(y_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
	//convert cart to polar
	double x = x_(0);
	double y = x_(1);
	double vx = x_(2);
	double vy = x_(3);
	
	double rho = sqrt(x*x+y*y);
	double theta = atan2(y,x);
	double rho_dot = (x*vx+y*vy)/rho;
	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, rho_dot;
	//If theta > 180 degree, theta = theta -180 degree
	//If theta < -180 degree, theta = theta + 180 degree

	//Update
  VectorXd y_ = z - z_pred;
	while (y_(1) > M_PI || y_(1) < -M_PI)
	{
		if (y_(1) > M_PI)
		{
			y_(1) -= M_PI;
		}
		else
		{
			y_(1) += M_PI;
		}
	}
  UpdateY(y_);
  
}

void KalmanFilter::UpdateY(const VectorXd &y_)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S_ = H_*P_*Ht + R_;
	MatrixXd Si = S_.inverse();
	MatrixXd K_ = P_*Ht*Si;
	//new state
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	x_ = x_ + (K_*y_);
	P_ = (I - K_*H_)*P_;
}
