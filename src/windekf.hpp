/*

 __        ___           _   _____ _  _______    ____ _               
 \ \      / (_)_ __   __| | | ____| |/ /  ___|  / ___| | __ _ ___ ___ 
  \ \ /\ / /| | '_ \ / _` | |  _| | ' /| |_    | |   | |/ _` / __/ __|
   \ V  V / | | | | | (_| | | |___| . \|  _|   | |___| | (_| \__ \__ \
    \_/\_/  |_|_| |_|\__,_| |_____|_|\_\_|      \____|_|\__,_|___/___/
                                                                      

*/

#ifndef __WIND_EKF_H__
#define __WIND_EKF_H__

#include "ekf.hpp"
#include <cmath>
#include <algorithm>

using namespace Eigen;
using namespace std;

class WindEKF : public EKF {

    public:

        /**
         * @brief State: [omega, P_max]^T
         *        Measurement: [omega_alternatore]^T
         */
        WindEKF(double J, double kt, int pairs);

        void set_inputs(double v_wind_api, double p_now);

        /**         
         * Prediction:                  
         * J * dw/dt = Tau_wind - Tau_load                      
         */
        Eigen::VectorXd f(const Eigen::VectorXd& x, double dt) override;


        Eigen::MatrixXd F(const Eigen::VectorXd& x, double dt) override;

        Eigen::VectorXd h(const Eigen::VectorXd& x_pred) override;

        Eigen::MatrixXd H(const Eigen::VectorXd& x) override;

    private:
        double _J;          // [kg*m^2]
        double _kt;         // fixed
        double _p_pairs;
        double _v_wind;     // [m/s]
        double _p_actual;   // [W]
};

#endif