/*
MPC Controller Implementation
*/

#include "MPCController.hpp"

MPCController::MPCController(const std::string& ObstacleFile) {
    // Load configuration from file
    std::cout << "MPCController initialized "<< std::endl;

    // Load YAML file
    YAML::Node config = YAML::LoadFile("../config.yaml");

    // Get horizon and dt from YAML
    cost_params.N = config["high_level_param"]["horizon"].as<int>();
    cost_params.dt = config["high_level_param"]["time_step"].as<double>();

    // Save the number of states and controls
    nx_ = 3; // e.g., [x, y, theta]
    nu_ = 2; // e.g., [v, omega]
    N_ = cost_params.N; // prediction horizon

    // Update the MPC Cost Params here
    // Update Q, R, Qf, S_delta as needed
    cost_params.Q = Eigen::Matrix3d::Identity();
    cost_params.R = Eigen::Matrix2d::Identity() * 0.1;
    cost_params.Qf = Eigen::Matrix3d::Identity() * 10.0;
    cost_params.S_delta = Eigen::Matrix2d::Identity() * 1.0;
    cost_params.w_speed_max = 10.0;

    // warm start buffers
    xpred_.assign(N_+1, Eigen::Vector3d::Zero());
    upred_.assign(N_, Eigen::Vector2d::Zero());

    u_prev_.setZero();

    // Obstacle Loader
    obstacle_loader_ = std::make_unique<ObstacleLoader>(ObstacleFile);

    // Get the number of obstacles
    this->num_obstacles_ = obstacle_loader_->getNumObstacles();

    std::cout <<"[MPCController] Number of obstacles loaded: " << num_obstacles_ << std::endl;
    
}

// Create a destructor to clean up obstacle_loader_
MPCController::~MPCController() {
    // unique_ptr will automatically clean up
}

void MPCController::start() {
    std::cout << "Starting MPC control loop..." << std::endl;
}

// Set the initial state, goal, and other parameters as needed
void MPCController::set_initial_state(const Eigen::Vector3d& x0) {
    x0_ = x0;
    xpred_[0] = x0;
}

// Set the goal state
void MPCController::set_goal_state(const Eigen::Vector3d& goal) {
    goal_ = goal;
}

// build constant references (linear interpolation)
void MPCController::buildReferences() {
    xref_.assign(N_+1, Eigen::Vector3d::Zero());
    uref_.assign(N_, Eigen::Vector2d::Zero());
    for (int k=0;k<=N_;++k) {
        double a = double(k)/double(N_);
        xref_[k] = (1.0-a)*x0_ + a*goal_;
    }
    for (int k=0;k<N_;++k) uref_[k].setZero();
}

// Set initial and final states and build reference trajectories
void MPCController::set_references(const Eigen::Vector3d& x0, const Eigen::Vector3d& goal) {
    this->set_initial_state(x0);
    this->set_goal_state(goal);
    this->buildReferences();
}

// Build dense H and g for cost only (not constraints)
void MPCController::build_mpc_cost_dense(const std::vector<double> &distances,
                                        int& n_var_out, int& slack_base, 
                                        Mat& H_out,
                                        Vec& g_out) {
  int N = N_;
  int nx = nx_;
  int nu = nu_;

  MPCCostParams& P = this->cost_params;

  // estimate slack count: upper bound K_per_step * N; in this cost-only builder we need slack variables to allocate space.
  int K_per_step = 1; // at least one slack per timestep (worst-case single aggregated slack)
  int n_slack = K_per_step * N;

  int n_u = N * nu;
  int n_x = N * nx; // x1..xN
  int n_var = n_u + n_x + n_slack;
//   std::cout << "[MPCController] Building cost: n_var=" << n_var << " (u:" << n_u << ", x:" << n_x << ", s:" << n_slack << ")" << std::endl;
  n_var_out = n_var;
//   std::cout << "[MPCController] N_var_out "<< n_var_out << std::endl;
  slack_base = n_u + n_x;

  // init
  H_out = Mat::Zero(n_var, n_var);
  g_out = Vec::Zero(n_var);

  auto u_idx = [&](int k)->int { return k * nu; };                      // 0..N-1
  auto x_idx = [&](int k)->int { return n_u + (k-1) * nx; };           // k in 1..N
  auto s_idx = [&](int k, int j)->int { return slack_base + k * K_per_step + j; };

  // 1) control cost R and u_ref linear terms
  for (int k = 0; k < N; ++k) {
    int ui = u_idx(k);
    H_out.block(ui, ui, nu, nu) += P.R;
    // linear term: g += -2 * R * u_ref
    Eigen::Vector2d uref = uref_[k];
    g_out.segment(ui, nu) += -2.0 * (P.R * uref);
  }

  // 2) state cost for x_{k+1}
  for (int k = 1; k <= N; ++k) {
    int xi = x_idx(k);
    // get Q or Qf for terminal
    if (k < N) {
      H_out.block(xi, xi, nx, nx) += P.Q;
      Eigen::Vector3d xr = xref_[k];
      g_out.segment(xi, nx) += -2.0 * (P.Q * xr);
    } else { // terminal k==N
      H_out.block(xi, xi, nx, nx) += P.Qf;
      Eigen::Vector3d xr = xref_[k];
      g_out.segment(xi, nx) += -2.0 * (P.Qf * xr);
    }
  }

  // 3) delta-u smoothness: (u_k - u_{k-1})^T S (u_k - u_{k-1})
  for (int k = 0; k < N; ++k) {
    int ui = u_idx(k);
    if (k == 0) {
      // u_prev known -> S on u0 and linear term
      H_out.block(ui, ui, nu, nu) += P.S_delta;
      g_out.segment(ui, nu) += -2.0 * (P.S_delta * u_prev_);
    } else {
      int uim1 = u_idx(k - 1);
      // add S to diag blocks
      H_out.block(ui, ui, nu, nu) += P.S_delta;
      H_out.block(uim1, uim1, nu, nu) += P.S_delta;
      // add off-diagonal -S (symmetric)
      H_out.block(ui, uim1, nu, nu) += -P.S_delta;
      H_out.block(uim1, ui, nu, nu) += -P.S_delta;
    }
  }

  // 4) speed shaping: w_k * (v_k - v_safe)^2 -> affects only v (first control)
  for (int k = 0; k < N; ++k) {
    int ui = u_idx(k);
    double d = distances[k];
    double ramp = 0.0;
    if (d < P.d_limit) ramp = (P.d_limit - d) / P.d_limit;
    double w_k = P.w_speed_max * std::max(0.0, ramp);
    // add to H at v component (index ui + 0)
    H_out(ui + 0, ui + 0) += w_k;
    // linear term: -2 * w_k * v_safe
    g_out(ui + 0) += -2.0 * w_k * P.v_safe;
  }

  // 5) slack diagonal penalty W_s (for each slack variable)
  for (int s = 0; s < n_slack; ++s) {
    int si = slack_base + s;
    H_out(si, si) += P.W_slack;
    // no linear term unless you want to bias slack use
  }

  // Note: There is no factor of 0.5 here — we built H and g so that the QP uses 0.5*z^T*H*z + g^T*z
  // (with symmetric H). This is the standard OSQP convention.
}


// Rollout Prediction using simple Euler integration
void MPCController::rolloutPrediction() {
    xpred_[0] = x0_;
    double dt = cost_params.dt;
    for (int k=0;k<N_;++k) {
        double th = xpred_[k][2];
        double v = upred_[k][0], w = upred_[k][1];
        Eigen::Vector3d xn;
        xn(0) = xpred_[k][0] + dt * v * std::cos(th);
        xn(1) = xpred_[k][1] + dt * v * std::sin(th);
        xn(2) = xpred_[k][2] + dt * w;
        xpred_[k+1] = xn;
    }    

}

void MPCController::buildConstraints(const std::vector<double> &distances,
                        const std::vector<std::vector<std::pair<Eigen::Vector2d,double>>> &obs_lin,
                        int n_var, int slack_base,
                        Eigen::SparseMatrix<double> &A_out,
                        Eigen::VectorXd &lb_out, Eigen::VectorXd &ub_out) {
    int N = N_;
    int nu = nu_, nx = nx_;
    int Kper = 1;
    int n_slack = Kper * N;
    int n_u = N*nu;
    int n_x = N*nx;

    // count constraints:
    // dynamics: N * nx equality rows
    // input bounds: 2 * N * nu (we implement variable bounds via A rows or via large box constraints) -> but OSQP supports only lb/ub on constraints; we'll implement each var bound as row.
    // obstacle inequalities: for each timestep, up to obs_lin[k].size() inequalities (we used nearestK). each uses one slack.
    int dyn_rows = N * nx;
    int bound_rows = n_u * 2; // lower and upper via two rows per variable
    int obs_rows = 0;
    for (int k=0;k<N;k++) obs_rows += (int)obs_lin[k].size();
    int total_rows = dyn_rows + bound_rows + obs_rows;

    std::vector<Eigen::Triplet<double>> triplets;
    lb_out = Eigen::VectorXd::Constant(total_rows, -1e20);
    ub_out = Eigen::VectorXd::Constant(total_rows,  1e20);

    // std::cout << "[MPCController] Building constraints: total_rows=" << total_rows 
    //           << " (dyn:" << dyn_rows << ", bound:" << bound_rows << ", obs:" << obs_rows << ")" << std::endl;

    int row = 0;
    // 1) Dynamics: x_{k+1} - A_k x_k - B_k u_k = 0
    for (int k=0;k<N;k++){
      // linearize around xpred_[k], upred_[k]
      Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
      Eigen::Matrix<double,3,2> B = Eigen::Matrix<double,3,2>::Zero();
      double dt = cost_params.dt;
      double theta = xpred_[k](2);
      double v = upred_[k](0);
      // continuous jacobians
      Eigen::Matrix3d df_dx = Eigen::Matrix3d::Zero();
      df_dx(0,2) = -v * std::sin(theta);
      df_dx(1,2) =  v * std::cos(theta);
      Eigen::Matrix<double,3,2> df_du;
      df_du << std::cos(theta), 0,
               std::sin(theta), 0,
               0,               1;
      A = Eigen::Matrix3d::Identity() + dt * df_dx;
      B = dt * df_du;

      // Build row blocks for this k (nx rows)
      // x_{k+1} variables at x_idx(k+1)
      int xkp1_idx = n_u + k*nx;
      int xk_idx = (k==0) ? -1 : (n_u + (k-1)*nx);
      int uk_idx = k*nu;

      // For each row r_i (0..2):
      for (int i=0;i<nx;i++){
        // x_{k+1} coefficient = 1
        triplets.emplace_back(row+i, xkp1_idx + i, 1.0);
        // -A_k * x_k  (if k==0 use x0 known -> move to rhs)
        if (k>0) {
          for (int j=0;j<nx;j++){
            double val = -A(i,j);
            if (std::abs(val) > 1e-12) triplets.emplace_back(row+i, xk_idx + j, val);
          }
        } else {
          // known x0 -> subtract A * x0 into RHS (we put into lb/ub)
        }
        // -B_k * u_k
        for (int j=0;j<nu;j++){
          double val = -B(i,j);
          if (std::abs(val) > 1e-12) triplets.emplace_back(row+i, uk_idx + j, val);
        }
        // set equality bounds lb=ub = rhs (we compute rhs below)
      }
      // compute rhs = - (if k==0) A*x0 + c (we have no c) else 0
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      if (k==0) {
        // move A*x0 to rhs: x1 - A x0 - B u0 = 0 -> lb = ub = -A*x0
        Eigen::Vector3d tmp = A * x0_;
        for (int i=0;i<nx;i++){
          lb_out[row+i] = -tmp(i);
          ub_out[row+i] = -tmp(i);
        }
      } else {
        for (int i=0;i<nx;i++){
          lb_out[row+i] = 0.0;
          ub_out[row+i] = 0.0;
        }
      }

      row += nx;
    }

    // std::cout << "[MPCController] Dynamics constraints built up to row " << row << std::endl;

    // 2) Input bounds: for each u variable, add lower and upper row:
    for (int k=0;k<N;k++){
      for (int j=0;j<nu;j++){
        int idx = k*nu + j;
        // lower bound row: u_idx >= u_min -> 1 * u_idx >= u_min
        triplets.emplace_back(row, idx, 1.0);
        lb_out[row] = (j==0) ? -1*cost_params.v_max : -1*cost_params.w_max;
        ub_out[row] = 1e20;
        row++;
        // upper bound row: u_idx <= u_max -> -1 * u_idx >= -u_max
        triplets.emplace_back(row, idx, -1.0);
        lb_out[row] = -1e20;
        ub_out[row] = (j==0) ? cost_params.v_max : cost_params.w_max;
        row++;
      }
    }
    // std::cout << "[MPCController] Input bounds built up to row " << row << std::endl;

    // 3) Obstacle inequalities: for each k and each linearized obs constraint add:
    // n^T p_{k+1} - s_i >= rhs  --> variables: px,py indices inside x block, slack index
    int slack_counter = 0;
    for (int k=0;k<N;k++){
      int xkp1_idx = n_u + k*nx;
      for (size_t cidx=0;cidx<obs_lin[k].size();++cidx){
        auto pr = obs_lin[k][cidx];
        Eigen::Vector2d n = pr.first;
        double rhs = pr.second;
        // n_x * px + n_y * py - 1 * s >= rhs
        triplets.emplace_back(row, xkp1_idx + 0, n.x());
        triplets.emplace_back(row, xkp1_idx + 1, n.y());
        int sidx = slack_base + slack_counter;
        triplets.emplace_back(row, sidx, -1.0);
        lb_out[row] = rhs;
        ub_out[row] = 1e20;
        row++;
      }
      slack_counter++;
    }

    // std::cout << "[MPCController] Obstacle constraints built up to row " << row << std::endl;

    // finalize A_out
    A_out.resize(total_rows, n_var);

    // for (int i=0;i<(int)triplets.size();++i){
    //     std::cout << "[MPCController] Triplet " << i << ": (" << triplets[i].row() << ", " << triplets[i].col() << ") = " << triplets[i].value() << std::endl;
    // }
    A_out.setFromTriplets(triplets.begin(), triplets.end());
    // std::cout << "[MPCController] Constraints matrix built with size " << A_out.rows() << "x" << A_out.cols() << std::endl;
  }

// Solve the MPC QP here
bool MPCController::solveMPC(Eigen::Vector2d& u0) {   

    // Dummy implementation - replace with actual QP solve
    u0.setZero();

    // Initialize the prediction rollout [upred_ is all zeros initially]
    rolloutPrediction();

    // Compute distances per step (for speed shaping) and gather obstacle linearizations
    std::vector<double> distances(N_, 10.0);
    std::vector<std::vector<std::pair<Eigen::Vector2d,double>>> obs_lin(N_);

    // get box obstacles
    for (int k=0;k<N_;++k) {
      Eigen::Vector2d p_k(xpred_[k].head<2>());
      // compute min distance and nearest obstacles
      std::vector<std::pair<double,int>> dist_idx;
      for (size_t i=0;i<num_obstacles_;++i) {
        double d = obstacle_loader_->sdfBox2d(p_k, i);
        dist_idx.emplace_back(d, (int)i);
      }
      std::sort(dist_idx.begin(), dist_idx.end(),
                [](auto &a, auto &b){ return a.first < b.first; });
      int K = std::min<int>(cost_params.nearestK, (int)dist_idx.size());
      double dmin = 1e9;
      for (int j=0;j<K;j++){
        int idx = dist_idx[j].second;
        double dij = dist_idx[j].first;
        dmin = std::min(dmin, dij);
        Eigen::Vector2d n; double rhs;
        obstacle_loader_->linearizeBoxConstraint(idx, p_k, cost_params.robot_inflation, n, rhs);
        obs_lin[k].push_back({n, rhs});
      }
      distances[k] = dmin;
    }

    // Build QP (dense H,g and constraints) then convert to sparse and call OSQP
    Eigen::MatrixXd H; Eigen::VectorXd g;
    int n_var, slack_base;
    build_mpc_cost_dense(distances, n_var, slack_base, H, g);

    Eigen::SparseMatrix<double> Hs = H.sparseView();

    // // Build constraints: dynamics equalities, input bounds, obstacle inequalities with slacks.
    Eigen::SparseMatrix<double> A; Eigen::VectorXd lb, ub;
    buildConstraints(distances, obs_lin, n_var, slack_base, A, lb, ub);

    // std::cout << "[MPCController] QP built: n_var=" << n_var<< std::endl;// << " n_cons=" << A.rows() << std::endl;
    // std::cout << "lb: " << lb.transpose() << std::endl;
    // std::cout << "ub: " << ub.transpose() << std::endl;

    // OSQP solve
    OsqpEigen::Solver solver;
    solver.data()->setNumberOfVariables(n_var);
    solver.data()->setNumberOfConstraints(A.rows());
    if (!solver.data()->setHessianMatrix(Hs)) { std::cerr<<"setHessian FAILED\n"; return false; }
    if (!solver.data()->setLinearConstraintsMatrix(A)) { std::cerr<<"setA FAILED\n"; return false; }
    solver.data()->setGradient(g);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);



    solver.settings()->setMaxIteration(2000);
    solver.settings()->setWarmStart(true);
    // Access the settings and set verbosity
    solver.settings()->setVerbosity(VERBOSITY);

    if (!solver.initSolver()) {
      std::cerr << "OSQP init failed\n";
      return false;
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      std::cerr << "OSQP solve failure\n";
      return false;
    }

    // Get the solution
    Eigen::VectorXd sol = solver.getSolution();

    // parse: u variables first: u0..u_{N-1}
    Eigen::Vector2d u0_local = sol.segment(0,2);
    u0 = u0_local;

    // warm-start update: shift u_pred and x_pred from solution
    for (int k=0;k<N_;++k) {
      upred_[k] = sol.segment(k*2,2);
    }
    u_prev_ = upred_[0];
    
    // recompute xpred by forward sim using upred_
    rolloutPrediction();

    return true;
}