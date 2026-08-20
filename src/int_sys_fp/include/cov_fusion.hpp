/**
 * @file cov_fusion.hpp
 * @brief Steady-state (SDRE-style) Riccati gain and Covariance Intersection fusion.
 *
 * Implements the `gain_mode: sdre_ci_experimental` option of pose_filter_params.yaml:
 * alongside the classic one-step Riccati update (gain recomputed every cycle from the
 * time-varying F_k/H_k), compute a second estimate using the STEADY-STATE gain obtained
 * by freezing the linearisation at the current operating point and solving the discrete
 * algebraic Riccati equation there, then fuse the two via Covariance Intersection.
 *
 * Terminology: strictly, SDRE refers to a state-dependent-coefficient factorisation
 * A(x)x. What is done here freezes the JACOBIAN at the operating point and solves the
 * DARE there - the standard "frozen-linearisation steady-state gain", sometimes called
 * SDREF. The `sdre` name is kept because it is what the YAML already documents.
 *
 * Header-only free functions with no ROS dependency, so the numerics can be exercised
 * without spinning a node.
 */

#ifndef INT_SYS_FP_COV_FUSION_HPP
#define INT_SYS_FP_COV_FUSION_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <algorithm>
#include <cmath>

namespace cov_fusion {

// ---------------------------------------------------------------------------------
// SPD inverse
// ---------------------------------------------------------------------------------

/**
 * @brief Inverse of a symmetric positive-definite matrix, with ridge escalation.
 *
 * The ridge is scaled by the diagonal magnitude on purpose: P entries are O(10) at
 * startup but O(1e-4) at steady state, so a fixed absolute ridge would be negligible in
 * one regime and a large relative perturbation in the other.
 *
 * @return false if no amount of ridging produced a finite inverse - callers must treat
 *         that as "fusion unavailable this cycle" rather than using a garbage result.
 */
template <int N>
inline bool spdInverse(const Eigen::Matrix<double, N, N>& M,
                       Eigen::Matrix<double, N, N>& out) {
    if (!M.allFinite()) return false;
    const double scale = std::max(1.0, M.diagonal().cwiseAbs().maxCoeff());
    double eps = 0.0;
    for (int attempt = 0; attempt < 4; ++attempt) {
        Eigen::LLT<Eigen::Matrix<double, N, N>> llt(
            M + eps * Eigen::Matrix<double, N, N>::Identity());
        if (llt.info() == Eigen::Success) {
            out = llt.solve(Eigen::Matrix<double, N, N>::Identity());
            if (out.allFinite()) return true;
        }
        eps = (eps == 0.0) ? 1e-12 * scale : eps * 100.0;
    }
    return false;
}

// ---------------------------------------------------------------------------------
// Discrete algebraic Riccati equation (filtering form)
// ---------------------------------------------------------------------------------

struct DareOptions {
    int    max_iters        = 500;
    double tol              = 1e-9;    ///< relative, on max|P_{k+1} - P_k|
    double divergence_trace = 1e12;
};

template <int N, int M>
struct DareResult {
    bool   converged  = false;
    int    iterations = 0;
    double residual   = 0.0;
    Eigen::Matrix<double, N, N> P_prior = Eigen::Matrix<double, N, N>::Zero();
    Eigen::Matrix<double, N, N> P_post  = Eigen::Matrix<double, N, N>::Zero();
    Eigen::Matrix<double, N, M> K       = Eigen::Matrix<double, N, M>::Zero();
};

/**
 * @brief Solve  P = A [P - P C'(C P C' + R)^-1 C P] A' + Q  by fixed-point iteration.
 *
 * Eigen has no DARE solver, and no eigenvalue reordering for a Schur/symplectic-pencil
 * approach, so this iterates the Riccati recursion to its fixed point. Warm-starting is
 * what makes that cheap: at 50 Hz the operating point barely moves between cycles, so
 * after the first solve the iteration count drops from hundreds to single digits.
 *
 * The posterior update uses the JOSEPH form, (I-KC)P(I-KC)' + KRK'. That is a sum of two
 * congruences of PSD matrices, hence PSD by construction even in floating point - unlike
 * the short form (I-KC)P, which can lose positive-definiteness and make a later Cholesky
 * fail. Explicit symmetrisation each iteration prevents asymmetry drift.
 *
 * Divergence is a real possibility, not paranoia: A = F(x,u,dt) is I + (nilpotent), so
 * all its eigenvalues sit on the unit circle, and the DARE only has a stabilising
 * solution if (A,C) is detectable. In a degenerate geometry - robot collinear with the
 * anchors and both neighbours, every range row parallel - rank(C) drops and P grows
 * without bound. Q > 0 does not save you. Hence the trace cap.
 *
 * @param P0 warm start (previous cycle's P_prior, or Q on the first call).
 */
template <int N, int Mm>
inline DareResult<N, Mm> solveFilterDare(const Eigen::Matrix<double, N, N>& A,
                                         const Eigen::Matrix<double, Mm, N>& C,
                                         const Eigen::Matrix<double, N, N>& Q,
                                         const Eigen::Matrix<double, Mm, Mm>& R,
                                         const Eigen::Matrix<double, N, N>& P0,
                                         const DareOptions& opt = DareOptions()) {
    using MatN  = Eigen::Matrix<double, N, N>;
    using MatM  = Eigen::Matrix<double, Mm, Mm>;
    using MatNM = Eigen::Matrix<double, N, Mm>;

    DareResult<N, Mm> res;
    if (!A.allFinite() || !C.allFinite() || !Q.allFinite() || !R.allFinite() ||
        !P0.allFinite()) {
        return res;  // converged == false
    }

    MatN P = 0.5 * (P0 + P0.transpose());
    const MatN I = MatN::Identity();

    MatNM K   = MatNM::Zero();
    MatN Ppost = MatN::Zero();

    for (int it = 1; it <= opt.max_iters; ++it) {
        const MatM S = C * P * C.transpose() + R;
        Eigen::LLT<MatM> llt(S);
        if (llt.info() != Eigen::Success) {
            res.iterations = it;
            return res;  // S not SPD - give up, caller falls back
        }
        // K = P C' S^-1, computed as (S^-1 C P)' since P is symmetric.
        K = llt.solve(C * P).transpose();

        const MatN IKC = I - K * C;
        Ppost = IKC * P * IKC.transpose() + K * R * K.transpose();   // Joseph

        MatN Pnext = A * Ppost * A.transpose() + Q;
        Pnext = 0.5 * (Pnext + Pnext.transpose());

        if (!Pnext.allFinite() || Pnext.trace() > opt.divergence_trace) {
            res.iterations = it;
            return res;
        }

        const double delta = (Pnext - P).cwiseAbs().maxCoeff();
        const double refmag = std::max(1.0, P.cwiseAbs().maxCoeff());
        P = Pnext;

        if (delta <= opt.tol * refmag) {
            res.converged  = true;
            res.iterations = it;
            res.residual   = delta;
            res.P_prior    = P;
            res.P_post     = 0.5 * (Ppost + Ppost.transpose());
            res.K          = K;
            return res;
        }
        res.residual = delta;
    }

    // Hit the iteration cap without converging: report failure rather than handing back
    // a partially-converged iterate, so the caller degrades to the classic estimate.
    res.iterations = opt.max_iters;
    return res;
}

// ---------------------------------------------------------------------------------
// Covariance Intersection
// ---------------------------------------------------------------------------------

struct CiResult {
    bool   ok = false;
    double w  = 0.5;                          ///< weight actually used
    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
};

/**
 * @brief Covariance Intersection of two pose estimates [x, y, theta].
 *
 * P_ci^-1 = w P_A^-1 + (1-w) P_B^-1
 * x_ci    = P_ci (w P_A^-1 x_A + (1-w) P_B^-1 x_B)
 *
 * Angle handling: the mean above cannot be evaluated directly, because averaging two
 * absolute thetas is wrong when they straddle +/-pi. Substituting x_B = x_A + d gives
 * the algebraically EXACT rewrite
 *
 *     x_ci = x_A + (1-w) * P_ci * P_B^-1 * d
 *
 * in which theta appears only inside the difference d - so wrapping d is enough and no
 * absolute angle is ever summed. Same trick the filters already use for innovation(5).
 * Secondary approximation worth stating: P_A(2,2) and P_B(2,2) are angular variances in
 * the tangent spaces at their respective means; treating them as living in one tangent
 * space is the standard first-order approximation, fine while |d(2)| stays small.
 *
 * @param w_fixed        weight used when minimize_trace is false; clamped to [0,1].
 * @param minimize_trace pick w minimising trace(P_ci) instead.
 *
 * Note on minimize_trace: trace(P_ci(w)) is convex on [0,1] (M(w) is affine and PD,
 * X -> X^-1 is matrix-convex on the PD cone, trace is linear and order-preserving),
 * hence unimodal, hence golden-section search finds the global minimum.
 *
 * CAVEAT for interpretation: CI is designed for fusing estimates whose cross-correlation
 * is UNKNOWN. Here both branches share the same prior and the same measurement, so they
 * are nearly perfectly correlated and CI is conservative by construction - P_ci is
 * guaranteed non-optimistic but is strictly larger than the true error covariance. In
 * closed loop that conservatism feeds back into the next predict and compounds. Expect
 * NIS/NEES to read "underconfident": correct behaviour, not a bug.
 */
inline CiResult fuseCI(const Eigen::Vector3d& xA, const Eigen::Matrix3d& PA_in,
                       const Eigen::Vector3d& xB, const Eigen::Matrix3d& PB_in,
                       double w_fixed, bool minimize_trace,
                       double (*wrapAngle)(double)) {
    CiResult res;
    if (!PA_in.allFinite() || !PB_in.allFinite() ||
        !xA.allFinite() || !xB.allFinite()) {
        return res;
    }

    // Symmetrise COPIES. PoseEKF::update stores the short form (I - K H) P_pred, which
    // is neither symmetrised nor Joseph, so P_A can arrive mildly asymmetric. The classic
    // path's own value is deliberately left untouched.
    const Eigen::Matrix3d PA = 0.5 * (PA_in + PA_in.transpose());
    const Eigen::Matrix3d PB = 0.5 * (PB_in + PB_in.transpose());

    double w = std::min(1.0, std::max(0.0, w_fixed));

    // Endpoint short-circuits. Without these, w = 1 still evaluates P_B^-1 and a singular
    // P_B would give NaN * 0 = NaN. They also make ci_weight:=1.0 reproduce the classic
    // filter bit-for-bit, which is the cleanest end-to-end regression test available.
    // Endpoint short-circuits. Without these, w = 1 would still evaluate P_B^-1 and a
    // singular P_B would give NaN * 0 = NaN. They also make ci_weight:=1.0 reproduce the
    // classic estimate exactly, which is the cleanest end-to-end regression test available.
    // The symmetrised copies are returned: both callers already store symmetric P, so this
    // is a no-op there, and it keeps the guarantee if some other caller does not.
    Eigen::Matrix3d IA, IB;
    if (!minimize_trace) {
        if (w >= 1.0 - 1e-12) { res.ok = true; res.w = 1.0; res.x = xA; res.P = PA; return res; }
        if (w <= 1e-12)       { res.ok = true; res.w = 0.0; res.x = xB; res.P = PB; return res; }
    }

    if (!spdInverse<3>(PA, IA) || !spdInverse<3>(PB, IB)) return res;

    auto fused_cov = [&](double ww, Eigen::Matrix3d& Pout) -> bool {
        const Eigen::Matrix3d Minf = ww * IA + (1.0 - ww) * IB;
        return spdInverse<3>(Minf, Pout);
    };

    Eigen::Matrix3d Pci;
    if (minimize_trace) {
        // Golden-section search on the convex trace(P_ci(w)).
        const double phi = 0.6180339887498949;
        double lo = 0.0, hi = 1.0;
        auto cost = [&](double ww) {
            Eigen::Matrix3d P;
            return fused_cov(ww, P) ? P.trace() : std::numeric_limits<double>::infinity();
        };
        double c = hi - phi * (hi - lo), d = lo + phi * (hi - lo);
        double fc = cost(c), fd = cost(d);
        for (int i = 0; i < 30; ++i) {
            if (fc < fd) { hi = d; d = c; fd = fc; c = hi - phi * (hi - lo); fc = cost(c); }
            else         { lo = c; c = d; fc = fd; d = lo + phi * (hi - lo); fd = cost(d); }
        }
        w = 0.5 * (lo + hi);
    }

    if (!fused_cov(w, Pci)) return res;
    Pci = 0.5 * (Pci + Pci.transpose());
    if (!Pci.allFinite() || Pci.diagonal().minCoeff() <= 0.0) return res;

    // Tangent-space mean: theta only ever appears inside the wrapped difference.
    Eigen::Vector3d d = xB - xA;
    d(2) = wrapAngle(xB(2) - xA(2));
    Eigen::Vector3d xci = xA + (1.0 - w) * (Pci * (IB * d));

    // Guard BEFORE wrapping: normalizeAngle is an unbounded while-loop, so a diverged
    // theta would spin it ~1e300 times and hang the node.
    if (!xci.allFinite() || std::abs(xci(2)) > 1e6) return res;
    xci(2) = wrapAngle(xci(2));

    res.ok = true;
    res.w  = w;
    res.x  = xci;
    res.P  = Pci;
    return res;
}

}  // namespace cov_fusion

#endif  // INT_SYS_FP_COV_FUSION_HPP
