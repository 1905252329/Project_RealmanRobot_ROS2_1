/*
 * Lemke Solver for LCP
 */

#ifndef LEMKE_H
#define LEMKE_H

#include <Eigen/Dense>

// Define a structure to store the result of the Lemke algorithm
struct LemkeResult {
    Eigen::VectorXd w;   // The w solution vector
    Eigen::VectorXd z;   // The z solution vector
    int status;          // Status: 1 for success, 0 for ray termination
    int iterations;      // Number of iterations performed
};

// Function declaration for the Lemke algorithm
// Takes q and M as inputs and returns a LemkeResult structure
LemkeResult Lemke(const Eigen::VectorXd &q, const Eigen::MatrixXd &M);

#endif // LEMKE_H

