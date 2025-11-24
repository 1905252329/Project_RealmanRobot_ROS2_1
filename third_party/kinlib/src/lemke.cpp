/*
 * Lemke Solver for LCP
 */

#include "kinlib/lemke.h"
#include <vector>
#include <limits>
#include <numeric>
#include <stdexcept>

LemkeResult Lemke(const Eigen::VectorXd &q, const Eigen::MatrixXd &M) {
    int n = q.size();

    // Check if M is square and matches the size of q
    if (M.rows() != n || M.cols() != n) {
        throw std::invalid_argument("Input M is not a square matrix or does not match the size of q");
    }

    // Initialization
    int dim = n;
    int ray = 0;
    int loop = 0;
    int maxloop = 1 << n; // 2^n for termination in case of cycling

    // Trivial solution check
    if (q.minCoeff() >= 0) {
        LemkeResult result;
        result.w = q;
        result.z = Eigen::VectorXd::Zero(n);
        result.status = 1;
        result.iterations = loop;
        return result;
    }

    // Set the initial tableau
    Eigen::MatrixXd table(dim, 2 * dim + 2);
    table.leftCols(dim) = Eigen::MatrixXd::Identity(dim, dim);
    table.middleCols(dim, dim) = -M;
    table.col(2 * dim) = -Eigen::VectorXd::Ones(dim);
    table.col(2 * dim + 1) = q;

    // Initialize row indices
    std::vector<int> index(dim);
    for (int i = 0; i < dim; ++i) index[i] = i;

    // Find the row of the most negative q_i
    int pivrow;
    q.minCoeff(&pivrow);

    // Replace index of the most negative q_i
    index[pivrow] = 2 * dim;

    // Entering variable
    int enter = pivrow + dim;

    // Perform Gauss-Jordan elimination
    table.row(pivrow) /= table(pivrow, 2 * dim);
    for (int i = 0; i < dim; ++i) {
        if (i != pivrow) {
            table.row(i) -= table.row(pivrow) * table(i, 2 * dim);
        }
    }

    // Iterative complementary pivoting
    while (*std::max_element(index.begin(), index.end()) == 2 * dim && loop < maxloop) {
        ++loop;

        // Pivot column
        Eigen::VectorXd pivcol = table.col(enter);

        // Check for ray termination
        std::vector<bool> postest(dim);
        for (int i = 0; i < dim; ++i) postest[i] = (pivcol(i) <= 0);
        if (std::accumulate(postest.begin(), postest.end(), 0) == dim) {
            ray = 1;
            break;
        }

        // Minimum ratio test
        Eigen::VectorXd ratio(dim);
        for (int i = 0; i < dim; ++i) {
            ratio(i) = postest[i] ? std::numeric_limits<double>::infinity() : table(i, 2 * dim + 1) / pivcol(i);
        }
        ratio.minCoeff(&pivrow);

        // Gauss-Jordan elimination
        table.row(pivrow) /= table(pivrow, enter);
        for (int i = 0; i < dim; ++i) {
            if (i != pivrow) {
                table.row(i) -= table.row(pivrow) * table(i, enter);
            }
        }

        // Update indices
        int drop = index[pivrow];
        index[pivrow] = enter;
        enter = (drop > dim) ? (drop - dim) : (drop + dim);
    }

    // Extract solution
    Eigen::VectorXd solution = Eigen::VectorXd::Zero(2 * dim + 1);
    for (int i = 0; i < dim; ++i) {
        solution(index[i]) = table(i, 2 * dim + 1);
    }

    LemkeResult result;
    result.w = solution.segment(0, dim);
    result.z = solution.segment(dim, dim);

    if (ray == 1) {
        result.status = 0; // Ray termination
    } else {
        result.status = 1; // Success
    }
    result.iterations = loop;

    return result;
}
