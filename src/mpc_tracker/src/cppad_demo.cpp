#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "Eigen/Core"
// #include <cppad/ipopt/solve.hpp>

using namespace std;

// namespace {
// using CppAD::AD;
// class FG_eval {
// public:
//     typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
//     void operator()(ADvector& fg, const ADvector& x)
//     {
//         assert(fg.size() == 3);
//         assert(x.size() == 4);
//         // variables
//         AD<double> x1 = x[0];
//         AD<double> x2 = x[1];
//         AD<double> x3 = x[2];
//         AD<double> x4 = x[3];
//         // f(x) objective function
//         fg[0] = x1 * x4 * (x1 + x2 + x3) + x3;
//         // constraints
//         fg[1] = x1 * x2 * x3 * x4;
//         fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
//         return;
//     }

// };

// }

// bool get_started(void)
// {
//     bool ok = true;
//     size_t i;
//     typedef CPPAD_TESTVECTOR(double) Dvector;

//     size_t nx = 4; // number of varibles
//     size_t ng = 2; // number of constraints
//     Dvector x0(nx); // initial condition of varibles
//     x0[0] = 0.0;
//     x0[1] = 0.0;
//     x0[2] = 0.0;
//     x0[3] = 0.0;

//     // lower and upper bounds for varibles
//     Dvector xl(nx), xu(nx);
//     for(i = 0; i < nx; i++)
//     {
//         xl[i] = 1.0;
//         xu[i] = 5.0;
//     }
//     Dvector gl(ng), gu(ng);
//     gl[0] = 25.0;    gu[0] = 1.0e19;
//     gl[1] = 40.0;    gu[1] = 40.0;
//     // object that computes objective and constraints
//     FG_eval fg_eval;

//     // options
//     string options;
//     // turn off any printing
//     options += "Integer print_level  0\n";
//     options += "String sb            yes\n";
//     // maximum iterations
//     options += "Integer max_iter     100\n";
//     // approximate accuracy in first order necessary conditions;
//     // see Mathematical Programming, Volume 106, Number 1,
//     // Pages 25-57, Equation (6)
//     options += "Numeric tol          1e-6\n";
//     // derivative testing
//     options += "String derivative_test   second-order\n";
//     // maximum amount of random pertubation; e.g.,
//     // when evaluation finite diff
//     options += "Numeric point_perturbation_radius   0.\n";


//     CppAD::ipopt::solve_result<Dvector> solution; // solution
//     CppAD::ipopt::solve<Dvector, FG_eval>(options, x0, xl, xu, gl, gu, fg_eval, solution); // solve the problem

//     cout<<"solution: "<<solution.x<<endl;

//     //
//     //check some of the solution values
//     //
//     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
//     //
//     double check_x[]  = {1.000000, 4.743000, 3.82115, 1.379408};
//     double check_zl[] = {1.087871, 0.,       0.,       0.      };
//     double check_zu[] = {0.,       0.,       0.,       0.      };
//     double rel_tol    = 1e-6; // relative tolerance
//     double abs_tol    = 1e-6; // absolute tolerance
//     for(i = 0; i < nx; i++)
//     {
//         ok &= CppAD::NearEqual(
//                     check_x[i], solution.x[i], rel_tol, abs_tol);
//         ok &= CppAD::NearEqual(
//                     check_zl[i], solution.zl[i], rel_tol, abs_tol);
//         ok &= CppAD::NearEqual(
//                     check_zu[i], solution.zu[i], rel_tol, abs_tol);
//     }

//     return ok;
// }

// int main()
// {
//     cout << "CppAD : Hello World Demo!" << endl;
//     get_started();
//     return 0;
// }

int main()
{
    std::cout << "TEST : Difference of quaternion Demo!" << std::endl;

    Eigen::Vector4d q(0.999,0,-0.044,0);
    Eigen::Vector3d w(0,-5*M_PI/180,0);
    Eigen::Vector4d q1(1,0,0,0);

    Eigen::Matrix4d w_hat;
    w_hat << 0, w.z(), -w.y(), w.x(),
              -w.z(), 0, w.x(), w.y(),
              w.y(), -w.x(), 0, w.z(),
              -w.x(), -w.y(), -w.z(), 0;

    q1 = q + 0.5 * w_hat * q;

    cout << q1 << endl;

    return 0;
}

