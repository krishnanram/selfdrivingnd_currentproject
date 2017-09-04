#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Context.h"
using CppAD::AD;

class FG_eval {
  public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    Context context;

    FG_eval(Eigen::VectorXd coeffs, Context context)
    {
        this->coeffs = coeffs;
        this->context = context ;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {

        size_t x_start = 0;
        size_t y_start = x_start + context.N;
        size_t psi_start = y_start + context.N;
        size_t v_start = psi_start + context.N;
        size_t cte_start = v_start + context.N;
        size_t epsi_start = cte_start + context.N;
        size_t delta_start = epsi_start + context.N;
        size_t a_start = delta_start + context.N - 1;

        // `fg` a vector of the cost constraints, 
        // `vars` is a vector of variable values (state & actuators)


        fg[0] = 0;

        // Define the costs
        for (unsigned int t = 0; t < context.N; t++) {
            fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);
            fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += CppAD::pow(vars[v_start + t] - context.ref_v, 2);
          }

        // Higher weights mean minimizing the use of actuators.
        for (unsigned int t = 0; t < context.N - 1; t++) {
            fg[0] += 0.001*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
          }

        // Minimize the value gap between sequential actuations.
        // Higher weights will influence the solver into keeping sequential values closer togther
        for (unsigned int t = 0; t < context.N - 2; t++) {
            fg[0] += 20000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 0.1*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
          }


        // Setup constraints
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (unsigned int t = 0; t < context.N-1; t++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + t + 1];
            AD<double> y1 = vars[y_start + t+ 1];
            AD<double> psi1 = vars[psi_start + t+ 1];
            AD<double> v1 = vars[v_start + t+ 1];
            AD<double> cte1 = vars[cte_start + t+ 1];
            AD<double> epsi1 = vars[epsi_start + t+ 1];

            // The state at time t.
            AD<double> x0 = vars[x_start + t];
            AD<double> y0 = vars[y_start + t];
            AD<double> psi0 = vars[psi_start + t];
            AD<double> v0 = vars[v_start + t];
            AD<double> cte0 = vars[cte_start + t];
            AD<double> epsi0 = vars[epsi_start + t];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + t];
            AD<double> a0 = vars[a_start + t];

            // Evalutate cte and desired psi
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
            AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0+2*coeffs[2]*x0+coeffs[1]);

            // constraint for variables and errors
            fg[2 + x_start + t ] = x1 - (x0 + v0 * CppAD::cos(psi0) * context.dt);
            fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * context.dt);
            fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / context.Lf * context.dt);
            fg[2 + v_start + t] = v1 - (v0 + a0 * context.dt);
            fg[2 + cte_start + t] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * context.dt));
            fg[2 + epsi_start + t] =
                epsi1 - ((psi0 - psides0) + v0 * delta0 / context.Lf * context.dt);
          }
      }
};


MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, Context context) {
    bool ok = true;
    // size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // state variables
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    size_t x_start = 0;
    size_t y_start = x_start + context.N;
    size_t psi_start = y_start + context.N;
    size_t v_start = psi_start + context.N;
    size_t cte_start = v_start + context.N;
    size_t epsi_start = cte_start + context.N;
    size_t delta_start = epsi_start + context.N;
    size_t a_start = delta_start + context.N - 1;
    
    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = context.N * 6 + (context.N - 1) * 2;

    // Set the number of constraints
    size_t n_constraints = context.N*6;

    // Initial value of the independent variables.
    // Zero besides initial state.
    Dvector vars(n_vars);
    for (unsigned int i = 0; i < n_vars; i++) {
        vars[i] = 0;
      }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Non-actuators upper and lower limits
    for (unsigned int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = - context.nactuators_limit;
        vars_upperbound[i] = context.nactuators_limit;
    }

    // The upper and lower limits of delta 
    // Set to -25 and 25 degrees (values in radians).
    for (unsigned int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = - context.delta_limit
        vars_upperbound[i] = context.delta_limit
    }

    // Acceleration/decceleration upper and lower limits.
    for (unsigned int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = - context.acc_limit
        vars_upperbound[i] = context.acc_limit
    }

    // Lower and upper limits for the constraints
    // Zero besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (unsigned int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
      }
    // Contraints initial states
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs, context);


    std::string options;
    options += "Integer print_level  0\n";

    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    // solve
    CppAD::ipopt::solve<Dvector, FG_eval>(
          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
          constraints_upperbound, fg_eval, solution);

    // Check the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values. 

    vector<double> result;
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    // Return predicted trajectory
    for (unsigned int i = 0; i < context.N-1; ++i)
    {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }

    // return result
    return result;
  }
