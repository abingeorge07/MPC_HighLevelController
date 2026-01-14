#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "ObstacleLoader.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <yaml-cpp/yaml.h>

#define VERBOSITY false

// types
using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;

struct MPCCostParams {
    double dt = 0.1;
    int N = 20;
    Eigen::Matrix3d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix3d Qf;
    Eigen::Matrix2d S_delta;         // smoothness weight on delta u
    double W_slack = 1e5;            // slack penalty
    double w_speed_max = 10.0;       // max weight for speed reduction
    double v_max = 5.0;              // max linear velocity
    double w_max = 4.0;              // max angular velocity
    double d_limit = 1.0;            // distance at which speed shaping begins
    double v_safe = 0.2;             // safe reduced speed near obstacles
    int nearestK = 3;                // number of nearest obstacles to consider
    double robot_inflation = 0.4;     // inflation for robot size in obstacle constraints
};

// struct MPCProblemData {
//   // references over horizon (size N -> x_ref[1..N], u_ref[0..N-1])
//   std::vector<Eigen::Vector3d> x_ref; // size N+1 (x0..xN) or at least x1..xN
//   std::vector<Eigen::Vector2d> u_ref; // size N
//   std::vector<double> distances;      // size N, detector distances for each step
//   Eigen::Vector2d u_prev;             // last applied control
// };

class MPCController {
public:
    MPCController(const std::string& ObstacleFile);
    ~MPCController();
    void start();
    void build_mpc_cost_dense(const std::vector<double> &distances,
                             int& n_var, int& slack_base, 
                             Mat& H_out,
                             Vec& g_out);

    // Solve function for MPC
    bool solveMPC(Eigen::Vector2d& u0);
    void set_references(const Eigen::Vector3d& x0, const Eigen::Vector3d& goal);
    void buildConstraints(const std::vector<double> &distances,
                        const std::vector<std::vector<std::pair<Eigen::Vector2d,double>>> &obs_lin,
                        int n_var, int slack_base,
                        Eigen::SparseMatrix<double> &A_out,
                        Eigen::VectorXd &lb_out, Eigen::VectorXd &ub_out);

    MPCCostParams cost_params;
private:
    int nx_; // number of states
    int nu_; // number of controls
    int N_;  // prediction horizon
    Eigen::Vector3d x0_, goal_;
    Eigen::Vector2d u_prev_; // last applied control
    std::vector<Eigen::Vector3d> xpred_;
    std::vector<Eigen::Vector2d> upred_;
    std::vector<Eigen::Vector3d> xref_;
    std::vector<Eigen::Vector2d> uref_;
    std::unique_ptr<ObstacleLoader> obstacle_loader_;
    // Number of obstacles
    int num_obstacles_;

    // First solve
    bool first_solve_ = true;

    double v_ref_ = 0.3; // nominal forward speed
    double w_ref_ = 0.0; // nominal angular speed

    
    // Private functions
    void buildReferences(void);
    void rolloutPrediction();
    void set_initial_state(const Eigen::Vector3d& x0);
    void set_goal_state(const Eigen::Vector3d& goal);
};

#endif // MPCCONTROLLER_H