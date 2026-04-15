/*

 __        ___           _   _____ _  _______    ____ _               
 \ \      / (_)_ __   __| | | ____| |/ /  ___|  / ___| | __ _ ___ ___ 
  \ \ /\ / /| | '_ \ / _` | |  _| | ' /| |_    | |   | |/ _` / __/ __|
   \ V  V / | | | | | (_| | | |___| . \|  _|   | |___| | (_| \__ \__ \
    \_/\_/  |_|_| |_|\__,_| |_____|_|\_\_|      \____|_|\__,_|___/___/
                                                                      

*/

#include "windekf.hpp"

WindEKF::WindEKF(double J, double kt, int pairs) 
    : EKF(2, 1), _J(J), _kt(kt), _p_pairs(static_cast<double>(pairs)){

    Q.resize(2, 2);
    R.resize(1, 1);

    Q << 0.01, 0, 
            0, 1.0; 

    R << 0.01; 
}

void WindEKF::set_inputs(double v_wind_api, double p_now){
    _v_wind = v_wind_api;
    _p_actual = p_now;
}

VectorXd WindEKF::f(const VectorXd& x, double dt){
    VectorXd x_new(2);
    double w = std::max(x(0), 0.1); 
    double p_max = x(1);

    // dw = (Tau_net / J) * dt -> Tau = P / omega
    double torque_net = (p_max / w) - (_p_actual / w);
    
    x_new(0) = w + (torque_net / _J) * dt;
    x_new(1) = _kt * std::pow(_v_wind, 3);
    
    return x_new;
}

MatrixXd WindEKF::F(const VectorXd& x, double dt){
    MatrixXd Fj(2, 2);
    double w = std::max(x(0), 0.1);
    double p_max = x(1);

    double df1_dw = 1.0 - ((p_max - _p_actual) / (_J * w * w)) * dt;
    double df1_dp = (1.0 / (_J * w)) * dt;

    Fj << df1_dw, df1_dp,
            0,      1; // assume dPmax/dw = 0
    
    return Fj;
}

VectorXd WindEKF::h(const VectorXd& x_pred){
    VectorXd z_pred(1);
    // freq = (omega * p_pairs) / 2pi
    z_pred(0) = (x_pred(0) * _p_pairs) / (2.0 * M_PI);
    return z_pred;
}

MatrixXd WindEKF::H(const VectorXd& x){
    MatrixXd Hj(1, 2);
    Hj << (_p_pairs / (2.0 * M_PI)), 0;
    return Hj;
}