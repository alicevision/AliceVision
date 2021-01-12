/*
File $Id: QuadProg++.cc 232 2007-06-21 12:29:00Z digasper $

 Author: Luca Di Gaspero
 DIEGM - University of Udine, Italy
 luca.digaspero@uniud.it
 http://www.diegm.uniud.it/digaspero/

 This software may be modified and distributed under the terms
 of the MIT license.  See the LICENSE file for details.

 */

#include "QuadProg++.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace quadprogpp {

// Utility functions for updating some data needed by the solution method
void compute_d(Eigen::VectorXd& d, const Eigen::MatrixXd& J, const Eigen::VectorXd& np);
void update_z(Eigen::VectorXd& z, const Eigen::MatrixXd& J, const Eigen::VectorXd& d, int iq);
void update_r(const Eigen::MatrixXd& R, Eigen::VectorXd& r, const Eigen::VectorXd& d, int iq);
bool add_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J, Eigen::VectorXd& d, unsigned int& iq, double& R_norm);
void delete_constraint(Eigen::MatrixXd& R,
                       Eigen::MatrixXd& J,
                       Eigen::VectorXi& A,
                       Eigen::VectorXd& u,
                       unsigned int n,
                       int p,
                       unsigned int& iq,
                       int l);

// Utility functions for computing the Cholesky decomposition and solving
// linear systems
void cholesky_decomposition(Eigen::MatrixXd& A);
void cholesky_solve(const Eigen::MatrixXd& L, Eigen::VectorXd& x, const Eigen::VectorXd& b);
void forward_elimination(const Eigen::MatrixXd& L, Eigen::VectorXd& y, const Eigen::VectorXd& b);
void backward_elimination(const Eigen::MatrixXd& U, Eigen::VectorXd& x, const Eigen::VectorXd& y);

// Utility functions for computing the scalar product and the euclidean
// distance between two numbers
double scalar_product(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
double distance(double a, double b);

// Utility functions for printing vectors and matrices
void print_matrix(const char* name, const Eigen::MatrixXd& A, int n = -1, int m = -1);

template<typename T>
void print_vector(const char* name, const Eigen::Matrix<T, Eigen::Dynamic, 1>& v, int n = -1);

// The Solving function, implementing the Goldfarb-Idnani method

double solve_quadprog(Eigen::MatrixXd& G,
                      Eigen::VectorXd& g0,
                      const Eigen::MatrixXd& CE,
                      const Eigen::VectorXd& ce0,
                      const Eigen::MatrixXd& CI,
                      const Eigen::VectorXd& ci0,
                      Eigen::VectorXd& x)
{
    std::ostringstream msg;
    unsigned int n = G.cols(), p = CE.cols(), m = CI.cols();
    if(G.rows() != n)
    {
        msg << "The matrix G is not a squared matrix (" << G.rows() << " x " << G.cols() << ")";
        throw std::logic_error(msg.str());
    }
    if(CE.rows() != n)
    {
        msg << "The matrix CE is incompatible (incorrect number of rows " << CE.rows() << " , expecting " << n << ")";
        throw std::logic_error(msg.str());
    }
    if(ce0.size() != p)
    {
        msg << "The vector ce0 is incompatible (incorrect dimension " << ce0.size() << ", expecting " << p << ")";
        throw std::logic_error(msg.str());
    }
    if(CI.rows() != n)
    {
        msg << "The matrix CI is incompatible (incorrect number of rows " << CI.rows() << " , expecting " << n << ")";
        throw std::logic_error(msg.str());
    }
    if(ci0.size() != m)
    {
        msg << "The vector ci0 is incompatible (incorrect dimension " << ci0.size() << ", expecting " << m << ")";
        throw std::logic_error(msg.str());
    }
    x.resize(n);

    int ip; // this is the index of the constraint to be added to the active set
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R(n, n), J(n, n);
    Eigen::VectorXd s(m + p), z(n), r(m + p), d(n), np(n), u(m + p), x_old(n), u_old(m + p);
    double f_value, psi, c1, c2, sum, ss, R_norm;
    const double inf = std::numeric_limits<double>::infinity();

    double t, t1, t2; /* t is the step lenght, which is the minimum of the partial step length t1
                       * and the full step length t2 */

    Eigen::VectorXi A(m + p), A_old(m + p), iai(m + p);
    unsigned int iq, iter = 0;
    Eigen::Matrix<bool, Eigen::Dynamic, 1> iaexcl(m + p);

    /*
     * Preprocessing phase
     */

    /* compute the trace of the original matrix G */
    c1 = 0.0;
    for(Eigen::Index i = 0; i < n; ++i)
    {
        c1 += G(i, i);
    }
    /* decompose the matrix G in the form L^T L */
    cholesky_decomposition(G);

    /* initialize the matrix R */
    for(Eigen::Index i = 0; i < n; ++i)
    {
        d[i] = 0.0;
        for(Eigen::Index j = 0; j < n; ++j)
        {
            R(i, j) = 0.0;
        }
    }
    R_norm = 1.0; /* this variable will hold the norm of the matrix R */

    /* compute the inverse of the factorized matrix G^-1, this is the initial value for H */
    c2 = 0.0;
    for(Eigen::Index i = 0; i < n; ++i)
    {
        d[i] = 1.0;
        forward_elimination(G, z, d);
        for(Eigen::Index j = 0; j < n; ++j)
        {
            J(i, j) = z[j];
        }
        c2 += z[i];
        d[i] = 0.0;
    }

    /* c1 * c2 is an estimate for cond(G) */

    /*
     * Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
     * this is a feasible point in the dual space
     * x = G^-1 * g0
     */
    cholesky_solve(G, x, g0);
    for(Eigen::Index i = 0; i < n; ++i)
    {
        x[i] = -x[i];
    }
    /* and compute the current solution value */
    f_value = 0.5 * scalar_product(g0, x);

    /* Add equality constraints to the working set A */
    iq = 0;
    std::cout << p << std::endl;
    for(Eigen::Index i = 0; i < p; ++i)
    {
        for(Eigen::Index j = 0; j < n; ++j)
        {
            np[j] = CE(j, i);
        }
        compute_d(d, J, np);
        update_z(z, J, d, iq);
        update_r(R, r, d, iq);

        /* compute full step length t2: i.e., the minimum step in primal space s.t. the contraint
          becomes feasible */
        t2 = 0.0;
        if(fabs(scalar_product(z, z)) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
            t2 = (-scalar_product(np, x) - ce0[i]) / scalar_product(z, np);

        /* set x = x + t2 * z */
        for(Eigen::Index k = 0; k < n; ++k)
        {
            x[k] += t2 * z[k];
        }

        /* set u = u+ */
        u[iq] = t2;
        for(Eigen::Index k = 0; k < iq; ++k)
        {
            u[k] -= t2 * r[k];
        }

        /* compute the new solution value */
        f_value += 0.5 * (t2 * t2) * scalar_product(z, np);
        A[i] = -i - 1;

        /*if (!add_constraint(R, J, d, iq, R_norm))
        {
          // Equality constraints are linearly dependent
          throw std::runtime_error("EQ Constraints are linearly dependent");
          return f_value;
        }*/
    }

    /* set iai = K \ A */
    for(Eigen::Index i = 0; i < m; ++i)
    {
        iai[i] = i;
    }

l1:
    iter++;
    /* step 1: choose a violated constraint */
    for(Eigen::Index i = p; i < iq; ++i)
    {
        ip = A[i];
        iai[ip] = -1;
    }

    /* compute s[x] = ci^T * x + ci0 for all elements of K \ A */
    ss = 0.0;
    psi = 0.0; /* this value will contain the sum of all infeasibilities */
    ip = 0;    /* ip will be the index of the chosen violated constraint */
    for(Eigen::Index i = 0; i < m; ++i)
    {
        iaexcl[i] = true;
        sum = 0.0;
        for(Eigen::Index j = 0; j < n; ++j)
        {
            sum += CI(j, i) * x[j];
        }
        sum += ci0[i];
        s[i] = sum;
        psi += std::min(0.0, sum);
    }

    if(fabs(psi) <= m * std::numeric_limits<double>::epsilon() * c1 * c2 * 100.0)
    {
        /* numerically there are not infeasibilities anymore */
        return f_value;
    }

    /* save old values for u and A */
    for(Eigen::Index i = 0; i < iq; ++i)
    {
        u_old[i] = u[i];
        A_old[i] = A[i];
    }
    /* and for x */
    for(Eigen::Index i = 0; i < n; ++i)
    {
        x_old[i] = x[i];
    }

l2: /* Step 2: check for feasibility and determine a new S-pair */
    for(Eigen::Index i = 0; i < m; ++i)
    {
        if(s[i] < ss && iai[i] != -1 && iaexcl[i])
        {
            ss = s[i];
            ip = i;
        }
    }
    if(ss >= 0.0)
    {
        return f_value;
    }

    /* set np = n[ip] */
    for(Eigen::Index i = 0; i < n; ++i)
        np[i] = CI(i, ip);
    /* set u = [u 0]^T */
    u[iq] = 0.0;
    /* add ip to the active set A */
    A[iq] = ip;

l2a: /* Step 2a: determine step direction */
    /* compute z = H np: the step direction in the primal space (through J, see the paper) */
    compute_d(d, J, np);
    update_z(z, J, d, iq);
    /* compute N* np (if q > 0): the negative of the step direction in the dual space */
    update_r(R, r, d, iq);

    /* Step 2b: compute step length */
    Eigen::Index l = 0;
    /* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
    t1 = inf; /* +inf */
    /* find the index l s.t. it reaches the minimum of u+[x] / r */
    for(Eigen::Index k = p; k < iq; ++k)
    {
        if(r[k] > 0.0)
        {
            if(u[k] / r[k] < t1)
            {
                t1 = u[k] / r[k];
                l = A[k];
            }
        }
    }
    /* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
    if(fabs(scalar_product(z, z)) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
    {
        t2 = -s[ip] / scalar_product(z, np);
        if(t2 < 0) // patch suggested by Takano Akio for handling numerical inconsistencies
            t2 = inf;
    }
    else
        t2 = inf; /* +inf */

    /* the step is chosen as the minimum of t1 and t2 */
    t = std::min(t1, t2);

    /* Step 2c: determine new S-pair and take step: */

    /* case (i): no step in primal or dual space */
    if(t >= inf)
    {
        /* QPP is infeasible */
        // FIXME: unbounded to raise
        return inf;
    }
    /* case (ii): step in dual space */
    if(t2 >= inf)
    {
        /* set u = u +  t * [-r 1] and drop constraint l from the active set A */
        for(Eigen::Index k = 0; k < iq; ++k)
            u[k] -= t * r[k];
        u[iq] += t;
        iai[l] = l;
        delete_constraint(R, J, A, u, n, p, iq, l);

        goto l2a;
    }

    /* case (iii): step in primal and dual space */

    /* set x = x + t * z */
    for(Eigen::Index k = 0; k < n; ++k)
        x[k] += t * z[k];
    /* update the solution value */
    f_value += t * scalar_product(z, np) * (0.5 * t + u[iq]);
    /* u = u + t * [-r 1] */
    for(Eigen::Index k = 0; k < iq; ++k)
        u[k] -= t * r[k];
    u[iq] += t;

    if(fabs(t - t2) < std::numeric_limits<double>::epsilon())
    {
        /* full step has taken */
        /* add constraint ip to the active set*/
        if(!add_constraint(R, J, d, iq, R_norm))
        {
            iaexcl[ip] = false;
            delete_constraint(R, J, A, u, n, p, iq, ip);

            for(Eigen::Index i = 0; i < m; ++i)
                iai[i] = i;
            for(Eigen::Index i = p; i < iq; ++i)
            {
                A[i] = A_old[i];
                u[i] = u_old[i];
                iai[A[i]] = -1;
            }
            for(Eigen::Index i = 0; i < n; ++i)
                x[i] = x_old[i];
            goto l2; /* go to step 2 */
        }
        else
            iai[ip] = -1;

        goto l1;
    }

    /* a patial step has taken */
    /* drop constraint l */
    iai[l] = l;
    delete_constraint(R, J, A, u, n, p, iq, l);

    /* update s[ip] = CI * x + ci0 */
    sum = 0.0;
    for(Eigen::Index k = 0; k < n; ++k)
        sum += CI(k, ip) * x[k];
    s[ip] = sum + ci0[ip];

    goto l2a;
}

inline void compute_d(Eigen::VectorXd& d, const Eigen::MatrixXd& J, const Eigen::VectorXd& np)
{
    const auto n = d.size();

    /* compute d = H^T * np */
    for(Eigen::Index i = 0; i < n; ++i)
    {
        double sum{0.0};
        for(Eigen::Index j = 0; j < n; ++j)
            sum += J(j, i) * np[j];
        d[i] = sum;
    }
}

inline void update_z(Eigen::VectorXd& z, const Eigen::MatrixXd& J, const Eigen::VectorXd& d, int iq)
{
    const auto n = z.size();

    /* setting of z = H * d */
    for(Eigen::Index i = 0; i < n; ++i)
    {
        z[i] = 0.0;
        for(Eigen::Index j = iq; j < n; ++j)
            z[i] += J(i, j) * d[j];
    }
}

inline void update_r(const Eigen::MatrixXd& R, Eigen::VectorXd& r, const Eigen::VectorXd& d, int iq)
{
    /* setting of r = R^-1 d */
    for(Eigen::Index i = iq - 1; i >= 0; --i)
    {
        double sum{0.0};
        for(Eigen::Index j = i + 1; j < iq; ++j)
            sum += R(i, j) * r[j];
        r[i] = (d[i] - sum) / R(i, i);
    }
}

bool add_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J, Eigen::VectorXd& d, unsigned int& iq, double& R_norm)
{
    const auto n = d.size();

    /* we have to find the Givens rotation which will reduce the element
      d[j] to zero.
      if it is already zero we don't have to do anything, except of
      decreasing j */
    for(Eigen::Index j = n - 1; j >= iq + 1; --j)
    {
        /* The Givens rotation is done with the matrix (cc cs, cs -cc).
        If cc is one, then element (j) of d is zero compared with element
        (j - 1). Hence we don't have to do anything.
        If cc is zero, then we just have to switch column (j) and column (j - 1)
        of J. Since we only switch columns in J, we have to be careful how we
        update d depending on the sign of gs.
        Otherwise we have to apply the Givens rotation to these columns.
        The i - 1 element of d has to be updated to h. */
        auto cc = d[j - 1];
        auto ss = d[j];
        const auto h = distance(cc, ss);
        if(fabs(h) < std::numeric_limits<double>::epsilon()) // h == 0
            continue;
        d[j] = 0.0;
        ss = ss / h;
        cc = cc / h;
        if(cc < 0.0)
        {
            cc = -cc;
            ss = -ss;
            d[j - 1] = -h;
        }
        else
            d[j - 1] = h;
        const auto xny = ss / (1.0 + cc);
        for(Eigen::Index k = 0; k < n; ++k)
        {
            const auto t1 = J(k, j - 1);
            const auto t2 = J(k, j);
            J(k, j - 1) = t1 * cc + t2 * ss;
            J(k, j) = xny * (t1 + J(k, j - 1)) - t2;
        }
    }
    /* update the number of constraints added*/
    iq++;
    /* To update R we have to put the iq components of the d vector
      into column iq - 1 of R
      */
    for(Eigen::Index i = 0; i < iq; ++i)
        R(i, iq - 1) = d[i];

    if(fabs(d[iq - 1]) <= std::numeric_limits<double>::epsilon() * R_norm)
    {
        // problem degenerate
        return false;
    }
    R_norm = std::max<double>(R_norm, fabs(d[iq - 1]));
    return true;
}

void delete_constraint(Eigen::MatrixXd& R,
                       Eigen::MatrixXd& J,
                       Eigen::VectorXi& A,
                       Eigen::VectorXd& u,
                       unsigned int n,
                       int p,
                       unsigned int& iq,
                       int l)
{
#ifdef TRACE_SOLVER
    std::cout << "Delete constraint " << l << ' ' << iq;
#endif
    Eigen::Index qq{0};

    bool found = false;
    /* Find the index qq for active constraint l to be removed */
    for(Eigen::Index i = p; i < iq; ++i)
    {
        if(A[i] == l)
        {
            qq = i;
            found = true;
            break;
        }
    }
    if(!found)
    {
        std::ostringstream os;
        os << "Attempt to delete non existing constraint, constraint: " << l;
        throw std::invalid_argument(os.str());
    }
    /* remove the constraint from the active set and the duals */
    for(Eigen::Index i = qq; i < iq - 1; ++i)
    {
        A[i] = A[i + 1];
        u[i] = u[i + 1];
        for(Eigen::Index j = 0; j < n; ++j)
        {
            R(j, i) = R(j, i + 1);
        }
    }

    A[iq - 1] = A[iq];
    u[iq - 1] = u[iq];
    A[iq] = 0;
    u[iq] = 0.0;

    for(Eigen::Index j = 0; j < iq; ++j)
    {
        R(j, iq - 1) = 0.0;
    }
    /* constraint has been fully removed */
    iq--;
#ifdef TRACE_SOLVER
    std::cout << '/' << iq << std::endl;
#endif

    if(iq == 0)
        return;

    for(Eigen::Index j = qq; j < iq; ++j)
    {
        auto cc = R(j, j);
        auto ss = R(j + 1, j);
        const auto h = distance(cc, ss);

        if(fabs(h) < std::numeric_limits<double>::epsilon()) // h == 0
            continue;

        cc = cc / h;
        ss = ss / h;
        R(j + 1, j) = 0.0;
        if(cc < 0.0)
        {
            R(j, j) = -h;
            cc = -cc;
            ss = -ss;
        }
        else
            R(j, j) = h;

        const auto xny = ss / (1.0 + cc);
        for(Eigen::Index k = j + 1; k < iq; ++k)
        {
            const auto t1 = R(j, k);
            const auto t2 = R(j + 1, k);
            R(j, k) = t1 * cc + t2 * ss;
            R(j + 1, k) = xny * (t1 + R(j, k)) - t2;
        }
        for(Eigen::Index k = 0; k < n; ++k)
        {
            const auto t1 = J(k, j);
            const auto t2 = J(k, j + 1);
            J(k, j) = t1 * cc + t2 * ss;
            J(k, j + 1) = xny * (J(k, j) + t1) - t2;
        }
    }
}

inline double distance(double a, double b)
{
    const auto a1 = fabs(a);
    const auto b1 = fabs(b);
    if(a1 > b1)
    {
        const auto t = (b1 / a1);
        return a1 * sqrt(1.0 + t * t);
    }
    else if(b1 > a1)
    {
        const auto t = (a1 / b1);
        return b1 * sqrt(1.0 + t * t);
    }
    return a1 * sqrt(2.0);
}

inline double scalar_product(const Eigen::VectorXd& x, const Eigen::VectorXd& y)
{
    return x.dot(y);
}

void cholesky_decomposition(Eigen::MatrixXd& A)
{
    const auto n = A.rows();

    for(Eigen::Index i = 0; i < n; ++i)
    {
        for(Eigen::Index j = i; j < n; ++j)
        {
            auto sum = A(i, j);
            for(Eigen::Index k = i - 1; k >= 0; --k)
                sum -= A(i, k) * A(j, k);
            if(i == j)
            {
                if(sum <= 0.0)
                {
                    std::ostringstream os;
                    // raise error
                    print_matrix("A", A);
                    os << "Error in cholesky decomposition, sum: " << sum;
                    throw std::logic_error(os.str());
                }
                A(i, i) = sqrt(sum);
            }
            else
                A(j, i) = sum / A(i, i);
        }
        for(Eigen::Index k = i + 1; k < n; ++k)
        {
            A(i, k) = A(k, i);
        }
    }
}

void cholesky_solve(const Eigen::MatrixXd& L, Eigen::VectorXd& x, const Eigen::VectorXd& b)
{
    const auto n = L.rows();
    Eigen::VectorXd y(n);

    /* Solve L * y = b */
    forward_elimination(L, y, b);
    /* Solve L^T * x = y */
    backward_elimination(L, x, y);
}

inline void forward_elimination(const Eigen::MatrixXd& L, Eigen::VectorXd& y, const Eigen::VectorXd& b)
{
    const auto n = L.rows();

    y[0] = b[0] / L(0, 0);
    for(Eigen::Index i = 1; i < n; ++i)
    {
        y[i] = b[i];
        for(Eigen::Index j = 0; j < i; ++j)
            y[i] -= L(i, j) * y[j];
        y[i] = y[i] / L(i, i);
    }
}

inline void backward_elimination(const Eigen::MatrixXd& U, Eigen::VectorXd& x, const Eigen::VectorXd& y)
{
    const auto n = U.rows();

    x[n - 1] = y[n - 1] / U(n - 1, n - 1);
    for(Eigen::Index i = n - 2; i >= 0; --i)
    {
        x[i] = y[i];
        for(Eigen::Index j = i + 1; j < n; ++j)
            x[i] -= U(i, j) * x[j];
        x[i] = x[i] / U(i, i);
    }
}

void print_matrix(const char* name, const Eigen::MatrixXd& A, int n, int m)
{
    std::ostringstream s;
    std::string t;
    if(n == -1)
        n = A.rows();
    if(m == -1)
        m = A.cols();

    s << name << ": " << std::endl;
    for(Eigen::Index i = 0; i < n; ++i)
    {
        s << " ";
        for(Eigen::Index j = 0; j < m; ++j)
            s << A(i, j) << ", ";
        s << std::endl;
    }
    t = s.str();
    t = t.substr(0, t.size() - 3); // To remove the trailing space, comma and newline

    std::cout << t << std::endl;
}

template<typename T>
void print_vector(const char* name, const Eigen::Matrix<T, Eigen::Dynamic, 1>& v, int n)
{
    std::ostringstream s;
    std::string t;
    if(n == -1)
        n = v.size();

    s << name << ": " << std::endl << " ";
    for(Eigen::Index i = 0; i < n; ++i)
    {
        s << v[i] << ", ";
    }
    t = s.str();
    t = t.substr(0, t.size() - 2); // To remove the trailing space and comma

    std::cout << t << std::endl;
}

} // namespace quadprogpp
