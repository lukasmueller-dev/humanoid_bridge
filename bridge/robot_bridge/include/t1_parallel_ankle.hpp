// t1_parallel_ankle.hpp ------------------------------------------------------
// Serial <-> parallel mapping of the Booster T1 two-crank ankle.
//
// Port of loco-mujoco-t1 (branch parallel_act): loco_mujoco/core/utils/parallel_ankle.py for the closed-form
// kinematics and parallel_act/ankle_jacobian.py for the Newton inverse, used by run_parallel_policy.py's Ankle class.
//
//   q_s = (ankle_pitch, ankle_roll)      serial angles, what the policy commands and observes
//   q_m = (crank_a, crank_b)             crank angles, joints ankle_drive_a/b of T1_23dof_parallel.xml
//   q_m = f(q_s),  dq_m = J dq_s,  tau_s = J^T tau_m
//
// Per crank, with d = anchor(q_s) - pivot, u = d_x t_x + d_z t_z, w = d_x t_z - d_z t_x,
// k = (|d|^2 + |t|^2 - L^2) / 2:   q = atan2(w, u) + branch * acos(k / sqrt(u^2 + w^2)),
// and J follows from the implicit function theorem in closed form.
//
// The geometry is T1_23dof_parallel.xml (booster_assets), identical to parallel_ankle.py's _GEOMETRY.
// No ROS dependencies so that tests/test_t1_parallel_ankle.cpp can build it standalone.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace sairol_bridge
{
namespace t1_parallel_ankle
{
    using Vec2 = std::array<double, 2>;
    using Vec3 = std::array<double, 3>;
    using Mat2 = std::array<std::array<double, 2>, 2>; // [row][col]

    enum Side
    {
        kLeft = 0,
        kRight = 1
    };

    struct Geometry
    {
        Vec3 p_pitch;                 // pitch link origin in the shank frame
        Vec3 p_roll;                  // roll link origin in the pitch link frame
        std::array<Vec3, 2> anchors;  // rod ends on the foot, roll link frame (a, b)
        std::array<Vec3, 2> pivots;   // crank pivots in the shank frame
        std::array<Vec3, 2> tips;     // crank tips in the crank frame
        Vec2 rod_lengths;
        Vec2 branch;                  // acos branch per crank, fixed at the neutral pose
    };

    inline const Geometry &geometry(Side side)
    {
        static const std::array<Geometry, 2> kGeometry = {{
            {{0.0, 0.0, -0.28},
             {0.0, 0.00025, -0.012},
             {{{-0.0415, 0.0275, 0.0327}, {-0.0415, -0.0275, 0.0327}}},
             {{{-0.014, 0.02775, -0.096}, {-0.014, -0.02775, -0.156}}},
             {{{0.04293593596233938, 0.0, 0.00234635952869361}, {0.04259337754140823, 0.0, -0.00589950757394721}}},
             {0.18, 0.11999999999999984},
             {-1.0, -1.0}},
            {{0.0, 0.0, -0.28},
             {0.0, -0.00025, -0.012},
             {{{-0.0415, -0.0275, 0.0327}, {-0.0415, 0.0275, 0.0327}}},
             {{{-0.014, -0.02775, -0.096}, {-0.014, 0.02775, -0.156}}},
             {{{0.04293593596233938, 0.0, 0.00234635952869361}, {0.04259337754140823, 0.0, -0.00589950757394721}}},
             {0.18, 0.11999999999999984},
             {-1.0, -1.0}},
        }};
        return kGeometry[side];
    }

    // E4315 crank motor peak torque (parallel_ankle.py CRANK_TORQUE_MAX, forcerange of T1_23dof_parallel.xml).
    constexpr double kCrankTorqueMax = 43.0;

    constexpr double kPi = 3.14159265358979323846;

    inline double wrap(double a)
    {
        double r = std::fmod(a + kPi, 2.0 * kPi);
        if (r < 0.0) r += 2.0 * kPi;
        return r - kPi;
    }

    // Crank angle i, its row of J (d theta_i / d q_s) and ratio = k / sqrt(u^2 + w^2) (|ratio| <= 1 when reachable).
    inline void crankTerms(Side side, int i, const Vec2 &q_s, double &theta, Vec2 &row, double &ratio)
    {
        const Geometry &g = geometry(side);
        const double cp = std::cos(q_s[0]), sp = std::sin(q_s[0]);
        const double cr = std::cos(q_s[1]), sr = std::sin(q_s[1]);
        const Vec3 &a = g.anchors[i];

        // inner = p_roll + Rx(qr) a ;  b = p_pitch + Ry(qp) inner
        const Vec3 inner = {g.p_roll[0] + a[0], g.p_roll[1] + cr * a[1] - sr * a[2], g.p_roll[2] + sr * a[1] + cr * a[2]};
        const Vec3 b = {g.p_pitch[0] + cp * inner[0] + sp * inner[2],
                        g.p_pitch[1] + inner[1],
                        g.p_pitch[2] - sp * inner[0] + cp * inner[2]};
        const Vec3 db_p = {-sp * inner[0] + cp * inner[2], 0.0, -cp * inner[0] - sp * inner[2]};
        const double v1 = -sr * a[1] - cr * a[2], v2 = cr * a[1] - sr * a[2]; // Rx'(qr) a = (0, v1, v2)
        const Vec3 db_r = {sp * v2, v1, cp * v2};

        const Vec3 d = {b[0] - g.pivots[i][0], b[1] - g.pivots[i][1], b[2] - g.pivots[i][2]};
        const double tx = g.tips[i][0], tz = g.tips[i][2];
        const double tt = g.tips[i][0] * g.tips[i][0] + g.tips[i][1] * g.tips[i][1] + g.tips[i][2] * g.tips[i][2];
        const double u = d[0] * tx + d[2] * tz;
        const double w = d[0] * tz - d[2] * tx;
        const double k = 0.5 * (d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + tt - g.rod_lengths[i] * g.rod_lengths[i]);
        const double r = std::sqrt(u * u + w * w);
        ratio = k / r;
        theta = std::atan2(w, u) + g.branch[i] * std::acos(std::clamp(ratio, -1.0, 1.0));

        const double c = std::cos(theta), s = std::sin(theta);
        double F_theta = -u * s + w * c; // zero on the workspace boundary, where d theta / d q_s diverges
        F_theta = (F_theta >= 0.0 ? 1.0 : -1.0) * std::max(std::abs(F_theta), 0.02 * r);
        const Vec2 dk = {d[0] * db_p[0] + d[1] * db_p[1] + d[2] * db_p[2], d[0] * db_r[0] + d[1] * db_r[1] + d[2] * db_r[2]};
        const Vec2 du = {tx * db_p[0] + tz * db_p[2], tx * db_r[0] + tz * db_r[2]};
        const Vec2 dw = {tz * db_p[0] - tx * db_p[2], tz * db_r[0] - tx * db_r[2]};
        for (int m = 0; m < 2; ++m)
        {
            row[m] = -(du[m] * c + dw[m] * s - dk[m]) / F_theta;
        }
    }

    // Forward map: crank angles q_m = f(q_s), J = dq_m/dq_s, and the loop-closure margin 1 - max|ratio|
    // (negative: q_s is outside the reachable workspace and q_m, J are evaluated on its boundary).
    inline void kinematics(Side side, const Vec2 &q_s, Vec2 &q_m, Mat2 &J, double &margin)
    {
        double ratio0, ratio1;
        crankTerms(side, 0, q_s, q_m[0], J[0], ratio0);
        crankTerms(side, 1, q_s, q_m[1], J[1], ratio1);
        q_m[0] = wrap(q_m[0]);
        q_m[1] = wrap(q_m[1]);
        margin = 1.0 - std::max(std::abs(ratio0), std::abs(ratio1));
    }

    inline Vec2 crankAngles(Side side, const Vec2 &q_s)
    {
        Vec2 q_m;
        Mat2 J;
        double margin;
        kinematics(side, q_s, q_m, J, margin);
        return q_m;
    }

    inline bool invert(const Mat2 &A, Mat2 &Ai)
    {
        const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (!std::isfinite(det) || std::abs(det) < 1e-9) return false;
        Ai = {{{A[1][1] / det, -A[0][1] / det}, {-A[1][0] / det, A[0][0] / det}}};
        return true;
    }

    inline Vec2 mul(const Mat2 &A, const Vec2 &x)
    {
        return {A[0][0] * x[0] + A[0][1] * x[1], A[1][0] * x[0] + A[1][1] * x[1]};
    }

    inline Vec2 mulT(const Mat2 &A, const Vec2 &x) // A^T x
    {
        return {A[0][0] * x[0] + A[1][0] * x[1], A[0][1] * x[0] + A[1][1] * x[1]};
    }

    // Inverse map q_s = f^-1(q_m): Newton with backtracking, warm-started at q_s_warm (ankle_jacobian.newton_inverse).
    // Returns false if the crank angles are not reached to within `tol_ok` (unreachable or bad warm start).
    inline bool serialAngles(Side side, const Vec2 &q_m, const Vec2 &q_s_warm, Vec2 &q_s,
                             int iters = 20, double tol = 1e-12, double tol_ok = 1e-6)
    {
        auto residual = [&](const Vec2 &x, Vec2 &r, Mat2 &J, double &margin)
        {
            Vec2 f;
            kinematics(side, x, f, J, margin);
            r = {wrap(f[0] - q_m[0]), wrap(f[1] - q_m[1])};
            return std::hypot(r[0], r[1]);
        };

        q_s = q_s_warm;
        Vec2 r;
        Mat2 J, Ji;
        double margin;
        double norm = residual(q_s, r, J, margin);
        for (int it = 0; it < iters && norm >= tol; ++it)
        {
            if (!invert(J, Ji)) return false;
            Vec2 step = mul(Ji, r);
            step = {-step[0], -step[1]};
            bool improved = false;
            for (int k = 0; k < 20; ++k)
            {
                const Vec2 cand = {q_s[0] + step[0], q_s[1] + step[1]};
                Vec2 r_new;
                Mat2 J_new;
                double margin_new;
                const double norm_new = residual(cand, r_new, J_new, margin_new);
                if (margin_new >= 0.0 && norm_new < norm)
                {
                    q_s = cand;
                    r = r_new;
                    J = J_new;
                    norm = norm_new;
                    improved = true;
                    break;
                }
                step = {0.5 * step[0], 0.5 * step[1]};
            }
            if (!improved) break;
        }
        return std::isfinite(norm) && norm < tol_ok;
    }

    // Motor command for one ankle, equivalent to the serial PD
    //     tau_s = Kp_s (q_ref - q_s) + Kd_s (dq_ref - dq_s) + tau_ff
    // at the measured crank state (run_parallel_policy.py Ankle, "torque" law, evaluated at the bridge rate).
    // The damping on the crank velocities, Kd_m = J^-T Kd_s J^-1, goes to the motor drivers as far as it is
    // diagonal (`kd`, applied in the motor loop); its off-diagonal part and the rest of the law go into `tau`:
    //     kd .* (0 - dq_m) + tau  ==  J^-T tau_s.
    struct CrankCommand
    {
        Vec2 q_s{0.0, 0.0};    // f^-1(q_m), serial state estimated from the crank encoders
        Vec2 dq_s{0.0, 0.0};   // J^-1 dq_m
        Vec2 tau{0.0, 0.0};    // feed-forward crank torque, clipped to +-tau_limit
        Vec2 tau_raw{0.0, 0.0};
        Vec2 kd{0.0, 0.0};     // crank-side damping gains (diagonal of Kd_m)
        Vec2 tau_s{0.0, 0.0};  // serial torque the law realises (before clipping)
        double margin{0.0};
        bool ok{false};
    };

    inline CrankCommand crankCommand(Side side, const Vec2 &q_m, const Vec2 &dq_m, const Vec2 &q_s_warm,
                                     const Vec2 &q_ref, const Vec2 &dq_ref, const Vec2 &kp_s, const Vec2 &kd_s,
                                     const Vec2 &tau_ff, double tau_limit)
    {
        CrankCommand out;
        if (!serialAngles(side, q_m, q_s_warm, out.q_s)) return out;
        Vec2 f;
        Mat2 J, Ji;
        kinematics(side, out.q_s, f, J, out.margin);
        if (!invert(J, Ji)) return out;
        out.dq_s = mul(Ji, dq_m);

        // Kd_m = Ji^T diag(kd_s) Ji
        Mat2 Kd_m{};
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b)
                Kd_m[a][b] = Ji[0][a] * kd_s[0] * Ji[0][b] + Ji[1][a] * kd_s[1] * Ji[1][b];

        Vec2 tau_s_no_damp;
        for (int j = 0; j < 2; ++j)
        {
            tau_s_no_damp[j] = kp_s[j] * (q_ref[j] - out.q_s[j]) + kd_s[j] * dq_ref[j] + tau_ff[j];
            out.tau_s[j] = tau_s_no_damp[j] - kd_s[j] * out.dq_s[j];
        }
        const Vec2 tau_m = mulT(Ji, tau_s_no_damp);
        out.kd = {Kd_m[0][0], Kd_m[1][1]};
        out.tau_raw = {tau_m[0] - Kd_m[0][1] * dq_m[1], tau_m[1] - Kd_m[1][0] * dq_m[0]};
        for (int j = 0; j < 2; ++j) out.tau[j] = std::clamp(out.tau_raw[j], -tau_limit, tau_limit);
        out.ok = std::isfinite(out.tau[0]) && std::isfinite(out.tau[1]) && std::isfinite(out.kd[0]) && std::isfinite(out.kd[1]);
        return out;
    }

} // namespace t1_parallel_ankle
} // namespace sairol_bridge
