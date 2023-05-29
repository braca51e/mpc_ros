#include "nmcp.hpp"

namespace nmpc_controller
{

    NMPCController::NMPCController():T_(50), nx_(3), nu_(2), dt_(0.1) {


    }

    void NMPCController::setUp() {

        // Define the symbolic variables
        x_ = opti_.variable(nx_, T_+1); // Decision variable state trajectory
        u_ = opti_.variable(nu_, T_); // Decision variable control trajectory
        p_ = opti_.parameter(nx_, 1); // Initial state

        _x_ref = opti_.parameter(3, T_+1); // Reference state trajectory
        _u_ref = opti_.parameter(2, T_); // Reference control trajectory

        // Objective function (quadratic) state and control
        Q = opti_.parameter(nx_, nx_); // State cost matrix
        R = opti_.parameter(nu_, nu_); // Control cost matrix

        // minimize the objective function using Q and R
        //J_ = 0;
        //for (int i = 0; i < T_; i++) {
        //    auto state_error = x_(casadi::Slice(), i) - _x_ref(casadi::Slice(), i) ;
        //    auto control_error = u_(casadi::Slice(), i) - _u_ref(casadi::Slice(), i);
        //    J_ += casadi::MX::mtimes({state_error.T(), Q, state_error}) + casadi::MX::mtimes({control_error.T(), R, control_error});
        //}
        
        J_ = casadi::MX::dot(x_ - _x_ref, casadi::MX::mtimes(Q, x_ - _x_ref))
                    //+ casadi::MX::dot(u_ - _u_ref, casadi::MX::mtimes(R, u_ - _u_ref));
                    + casadi::MX::dot(u_, casadi::MX::mtimes(R, u_));

        opti_.minimize(J_);

        // Define the control horizons
        for (int i = 0; i < T_; i++) {
            opti_.subject_to(x_(casadi::Slice(), i+1) == x_(casadi::Slice(), casadi::Slice(i)) + \
                                dynamics_(x_(casadi::Slice(), casadi::Slice(i)), u_(casadi::Slice(), casadi::Slice(i)), dt_));
        }

        // add constraints in change of velocity
        for (int i = 0; i < T_-1; i++) {
            auto dvel = u_(casadi::Slice(), i+1) - u_(casadi::Slice(), i)/dt_;
            opti_.subject_to(opti_.bounded(-MAX_DELTA_VELOCITY, dvel(0), MAX_DELTA_VELOCITY));
            opti_.subject_to(opti_.bounded(-MAX_DELTA_OMEGA, dvel(1), MAX_DELTA_OMEGA));
        }

        opti_.subject_to(opti_.bounded(-MAX_LINEAR_VELOCITY, u_(0), MAX_LINEAR_VELOCITY));
        opti_.subject_to(opti_.bounded(-MAX_ANGULAR_VELOCITY, u_(1), MAX_ANGULAR_VELOCITY));
        // @TODO Add x constraints SQUARE MAP GAZEBO
        //opti_.subject_to(opti_.bounded(-1.77, x_(0), 1.77));
        //opti_.subject_to(opti_.bounded(-1.77, x_(1), 1.77));
        opti_.subject_to(opti_.bounded(-2.07, x_(0), 2.07));
        opti_.subject_to(opti_.bounded(-2.9, x_(1), 2.9));
        opti_.subject_to(opti_.bounded(-M_PI, x_(3), M_PI));
        opti_.subject_to(x_(casadi::Slice(), 0) == p_);

        solver_options_["ipopt.print_level"] = 0;
        solver_options_["ipopt.sb"] = "yes";
        solver_options_["ipopt.max_iter"] = 2000;
        solver_options_["ipopt.tol"] = 1e-8;
        solver_options_["print_time"] = 0;
        solver_options_["ipopt.acceptable_obj_change_tol"] = 1e-6;
        opti_.solver("ipopt", solver_options_);

        //paper values
        opti_.set_value(Q, casadi::DM::diag(casadi::DM::vertcat({1.0, 5.0, 0.5})));               
        opti_.set_value(R, casadi::DM::diag(casadi::DM::vertcat({0.5, 0.25})));

        //opti_.set_value(Q, casadi::DM::eye(3));
        //opti_.set_value(R, casadi::DM::eye(2));
        opti_.set_value(_x_ref, casadi::DM::zeros(3, T_+1));
        opti_.set_value(_u_ref, casadi::DM::zeros(2, T_));

        u_init = casadi::DM::repmat({0, 0}, 1, T_);
        x_init = casadi::DM::repmat({0.0, 0.0, 0.0}, 1, T_+1);

    }

    std::vector<double> NMPCController::solve(const std::vector<double> x0) {

        // Set the initial guess
        opti_.set_initial(x_, x_init);
        opti_.set_initial(u_, u_init);

        // Set the initial state
        opti_.set_value(p_, x0);

        // Solve the optimization problem
        casadi::OptiSol solution = opti_.solve();

        // Get the optimal control u and push in the control vector
        std::vector<double> control;
        casadi::Matrix<double> u0 = solution.value(u_)(casadi::Slice(), 0);
        control = u0.get_elements();
         
        // warm start
        x_init = casadi::DM::repmat(solution.value(x_)(casadi::Slice(), 0), 1, T_+1);
        u_init = solution.value(u_)(casadi::Slice(), casadi::Slice());

        return control;
    }

    casadi::MX NMPCController::dynamics_(const casadi::MX& x, const casadi::MX& u, const double dt) {
        // The dynamics function
        casadi::MX xdot(nx_, 1);

        xdot(0) = u(0)*cos(x(2));
        xdot(1) = u(0)*sin(x(2));
        xdot(2) = u(1);

        return xdot*dt;
    }

    casadi::MX NMPCController::dynamics_2(const casadi::MX& x, const casadi::MX& u, const double dt) {
        // The dynamics function
        casadi::MX xdot(nx_, 1);

        xdot(0) = u(0)*cos(x(2)) - sin(x(2))*u(1);
        xdot(1) = u(0)*sin(x(2)) + cos(x(2))*u(1);
        xdot(2) = u(1);

        return xdot*dt;
    }

    void NMPCController::setReference(const std::vector<double> x_ref, const std::vector<double> u_ref) {

        // Addign each colum of the reference trajectory to the decision variable
        opti_.set_value(_x_ref, casadi::DM::repmat(x_ref, 1, T_+1));
        opti_.set_value(_u_ref, casadi::DM::repmat(u_ref, 1, T_));

    }

    void NMPCController::setDt(double dt) {
        dt_ = dt;
    }

}