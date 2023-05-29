#ifndef NMPC_H
#define NMPC_H

#include <casadi/casadi.hpp>

// Turtlebot constraints
#define MAX_LINEAR_VELOCITY     0.12   // m/s
#define MAX_ANGULAR_VELOCITY    2.84   // rad/s
#define MAX_DELTA_VELOCITY    1.5   // m/s
#define MAX_DELTA_OMEGA    M_PI/2.0   // rad/s

namespace nmpc_controller
{

class NMPCController {
    public:
      NMPCController();
      void setUp();
      std::vector<double> solve(const std::vector<double> x0);
      void setReference(const std::vector<double> x_ref, const std::vector<double> u_ref);
      void setDt(double dt);
    
    private:
      casadi::Opti opti_;
      int T_; // horizon
      int nx_; // state size
      int nu_; // control size
      double dt_; // sampling time
      casadi::Dict solver_options_;
      casadi::MX Q;
      casadi::MX R;
      casadi::MX p_;
      casadi::MX x_;
      casadi::MX _x_ref;
      casadi::DM x_init;
      casadi::MX u_;
      casadi::MX _u_ref;
      casadi::DM u_init;
      casadi::MX J_;

      casadi::MX dynamics_(const casadi::MX& x, const casadi::MX& u, const double dt);
      casadi::MX dynamics_2(const casadi::MX& x, const casadi::MX& u, const double dt);
    
  };
}

#endif