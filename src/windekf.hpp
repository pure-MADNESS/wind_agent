#ifndef __WIND_EKF_H__
#define __WIND_EKF_H__

#include "ekf.hpp"

class WindEKF : public EKF{

  public:

    WindEKF() : EKF(2, 2) {
        Q << 0.001, 0, 0, 0.001; 
        R << 0.01, 0, 0, 0.1;
    }

    Eigen::VectorXd f(const Eigen::VectorXd& x_old, double dt) override {
        Eigen::VectorXd x_new(2);
        x_new(0) = x_old(0) + x_old(1) * dt; // theta = theta + omega*dt
        x_new(1) = x_old(1);                // omega costante (modello random walk)
        return x_new;
    }

    Eigen::MatrixXd F(const Eigen::VectorXd& x, double dt) override {
        Eigen::MatrixXd F_j(2, 2);
        F_j << 1, dt,
               0, 1;
        return F_j;
    }

    Eigen::VectorXd h(const Eigen::VectorXd& x_pred) override {
        Eigen::VectorXd z_pred(2);
        z_pred(0) = x_pred(0); // encoder -> theta
        z_pred(1) = x_pred(1); // gyro -> omega
        return z_pred;
    }

    // Jacobiana del modello di misura
    Eigen::MatrixXd H(const Eigen::VectorXd& x) override {
        return Eigen::MatrixXd::Identity(2, 2);
    }

}; 



#endif
