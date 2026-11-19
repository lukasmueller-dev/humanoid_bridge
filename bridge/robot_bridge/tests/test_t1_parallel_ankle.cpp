// Test of include/t1_parallel_ankle.hpp, no ROS. Run: build/robot_bridge/test_t1_parallel_ankle
// Reference values come from loco-mujoco-t1 (branch parallel_act) parallel_act/ankle_jacobian.py, AnkleGeometry read
// from booster_assets robots/T1/T1_23dof_parallel.xml; tau_m = J^-T tau_s is what run_parallel_policy.py's Ankle
// class sends to the cranks for the serial PD (kp 50, kd 3) used in training.
#include "t1_parallel_ankle.hpp"

#include <cstdio>
#include <random>

namespace pa = sairol_bridge::t1_parallel_ankle;

namespace
{
struct Reference
{
    int side;
    pa::Vec2 q_s, q_m;
    std::array<double, 4> J; // row-major
    pa::Vec2 dq_m, q_ref, dq_ref, tau_ff, tau_m;
};

// clang-format off
const Reference kReference[] = {
    {0, {0, 0}, {-4.49840292660042e-06, 2.0549575068251613e-05}, {-1.1994339764279227, -0.65572787694912937, -1.2040423609231194, 0.58352816737257573}, {-1.2382434462261214, 0.99449807308362126}, {-0.14233615491214652, 0.17945977885489756}, {-0.37633709597902909, -0.15334710205484869}, {1.310810375281767, -0.3632034545233549}, {0.5542015737892616, 5.3255102688886939}},
    {0, {-0.25, 0}, {0.28380214584641283, 0.27543957851664969}, {-1.0749452998616125, -0.51089951132776446, -1.0095761350465844, 0.41393932928372429}, {0.75223726683364789, -0.98252186925618823}, {-0.14859475653007737, 0.015257325287711287}, {-0.34053656700181567, 0.57685740685680864}, {-0.78722068283342006, -0.18600844207739398}, {-9.5308429939748542, 7.5069398725059902}},
    {0, {-0.59999999999999998, 0.29999999999999999}, {0.56003362828355385, 0.70238961930549815}, {-1.0510185198066018, -0.14704772993325327, -0.73024362305343304, 0.45953091053503548}, {1.5955038762927325, 0.890864557326978}, {-0.71861790372954015, 0.2049253361767398}, {0.50072934526010515, -0.43918248402792015}, {-0.059236102273459768, 1.9229487992049545}, {3.7682306854213223, -5.2914349339856788}},
    {0, {0.29999999999999999, -0.34999999999999998}, {-0.049270007770172874, -0.64943646411947409}, {-1.1018813664335838, -0.99716120174763301, -1.9404468742234537, 0.41863350769979502}, {-2.9313730674426521, -3.2068666280240525}, {0.31649074221897366, -0.43924351838185166}, {-0.67869598244974627, 0.93985082643226514}, {0.064274342191514844, -1.5365375501169187}, {5.9271528075607796, 0.080467847686499727}},
    {0, {-0.80000000000000004, -0.20000000000000001}, {0.89478636768459019, 0.73304950331823981}, {-0.85587333969149315, -0.42963275937430323, -0.85133237903828551, 0.060194396524997876}, {-0.89825487750689548, -0.35390421708841968}, {-0.75479867957878388, -0.033080918083638944}, {-0.92081424667159428, 0.057178526520043294}, {-0.16265646845838511, -1.7506016834004976}, {-6.7222337236086807, 9.2799135515070326}},
    {0, {0.20000000000000001, 0.40000000000000002}, {-0.49208754324835979, 0.086019104565669302}, {-1.6244232114413941, -0.32980421045237479, -1.0273019302051056, 0.91754714690373573}, {-1.3835062130208353, 0.71348221561303582}, {0.23717640724171363, 0.30403897909488931}, {0.67976304206281757, 0.01899176304301875}, {0.043555537866132088, 1.0121208308087115}, {3.3440928566847039, -7.4738972334538696}},
    {1, {0, 0}, {-4.49840292660042e-06, 2.0549575068251613e-05}, {-1.1994339764279227, 0.65572787694912937, -1.2040423609231194, -0.58352816737257573}, {2.5275296903015865, 0.94962235844602316}, {0.073314762401302858, 0.11483877662192042}, {-0.61676748195972952, 0.60472832226906004}, {-1.2347042957711989, -1.6737895305459491}, {-0.22813575569824154, -3.7640147219555562}},
    {1, {-0.25, 0}, {0.28380214584641283, 0.27543957851664969}, {-1.0749452998616125, 0.51089951132776446, -1.0095761350465844, -0.41393932928372429}, {-0.78908001898396363, -2.0327124954163831}, {-0.099385161433367786, -0.01123611225648391}, {-0.45190322277256345, -0.98581634279366748}, {0.58288358229979131, 0.87963753403477218}, {-8.4045137367830129, 6.4771740942615033}},
    {1, {-0.59999999999999998, 0.29999999999999999}, {0.77483456586303667, 0.54981662355148853}, {-0.84440377621363683, 0.56421287452328583, -0.93509258466435374, -0.04730708091843043}, {-1.6256930064551962, -1.2138782508802561}, {-0.71391273313481052, 0.35573255202663512}, {0.61010966629001939, 0.92734174568994177}, {-1.3979006783152901, -0.071150447202653844}, {14.144846304589581, -2.8381389638765122}},
    {1, {0.29999999999999999, -0.34999999999999998}, {-0.63841104695526596, -0.069630511381929328}, {-1.7082341738289151, 0.48832606843139298, -1.1337272219487831, -0.99333764913045319}, {-2.8480258949736017, -1.4829276477377484}, {0.33580082483361923, -0.54020372900265468}, {0.34691977430587784, 0.83817723926764498}, {1.3073013182268842, 1.5420810668397871}, {-2.0159897713617001, 3.5654706059803964}},
    {1, {-0.80000000000000004, -0.20000000000000001}, {0.77799240359905841, 0.80897895002742537}, {-0.98103789096708016, 0.16013499939658363, -0.73211777072302719, -0.32628866327855183}, {-0.7922427673702146, -0.13750245245858606}, {-0.6925932004414983, -0.31533010295699582}, {0.66254966932892234, -0.8745641548584635}, {1.3019512535742233, -1.3419709341035948}, {-16.205817662953731, 12.515534897032088}},
    {1, {0.20000000000000001, 0.40000000000000002}, {0.10637361354507568, -0.47926387818911476}, {-1.0387984327534283, 0.94942957540382278, -1.7896069161524186, -0.24135669484100325}, {-0.17718840139880299, 1.0706770759293018}, {0.27653481411109654, 0.27142875126974875}, {-0.2074876755660271, -0.98835080978403811}, {-0.950021148999594, -0.31524474308417894}, {-7.3571317085015862, 2.1737088079755931}},
};
// clang-format on


// Impedance transfer: loco-mujoco-t1 parallel_act/impedance_transfer.py impedance_transfer(..) with Kps = 50, Kds = 3, dq_ref = 0,
// tau_ff = 0; literal = 1 uses the printed eq. 29 / 30 terms. Kp_m, Kd_m row-major.
struct ImpedanceReference
{
    int side, literal;
    pa::Vec2 q_s, dq_m, q_ref;
    std::array<double, 4> Kp, Kd;
    pa::Vec2 q_star, tau_m;
};

// clang-format off
const ImpedanceReference kImpedanceReference[] = {
    {0, 0, {-0.25, 0}, {0.75223726683364789, -0.98252186925618823}, {-0.34053656700181567, 0.57685740685680864}, {106.2370744215138, -47.845342598656501, -58.964907895270549, 29.93273367662399}, {3.869536132381449, -2.8398002715206156, -2.8398002715206161, 4.6038506995500885}, {2.5425047792121247, 5.8834283435709001}, {-34.059159151720344, 41.337829016354675}},
    {0, 1, {-0.25, 0}, {0.75223726683364789, -0.98252186925618823}, {-0.34053656700181567, 0.57685740685680864}, {17.874545799248743, -16.345054546622425, 41.572076522012551, 71.705305791877635}, {3.869536132381449, -2.8398002715206156, -2.8398002715206161, 4.6038506995500885}, {-0.46401361960354931, 1.1926171140701931}, {-34.059159151720344, 41.337829016354675}},
    {0, 0, {-0.59999999999999998, 0.29999999999999999}, {1.5955038762927325, 0.890864557326978}, {0.50072934526010515, -0.43918248402792015}, {-10.009123916042416, 47.291993654221194, 44.792252812877059, 15.966118819447335}, {6.4078694226838415, -6.0248403771102872, -6.0248403771102872, 9.6946660299879799}, {-1.1106380252591161, 0.40962591146488997}, {-1.9798835572059872, -78.531425403709463}},
    {0, 1, {-0.59999999999999998, 0.29999999999999999}, {1.5955038762927325, 0.890864557326978}, {0.50072934526010515, -0.43918248402792015}, {157.65669739528713, -47.672412976382994, -195.45827593516762, -40.70884218255248}, {6.4078694226838415, -6.0248403771102872, -6.0248403771102872, 9.6946660299879799}, {0.80834430275571023, 1.4632334367414146}, {-1.9798835572059872, -78.531425403709463}},
    {0, 0, {0.29999999999999999, -0.34999999999999998}, {-2.9313730674426521, -3.2068666280240525}, {-0.67869598244974627, 0.93985082643226514}, {115.77005284487397, -39.158315606580203, -37.908670862747343, -4.5145340395567057}, {2.058865151863623, -0.89902404061767238, -0.89902404061767238, 1.1538755171379305}, {-1.1234020830279787, -2.7096843275819489}, {-40.524239387703069, 51.084928442746765}},
    {0, 1, {0.29999999999999999, -0.34999999999999998}, {-2.9313730674426521, -3.2068666280240525}, {-0.67869598244974627, 0.93985082643226514}, {-15.668379087876719, -15.926882432635537, 34.947576210471084, 71.37187731126987}, {2.058865151863623, -0.89902404061767238, -0.89902404061767238, 1.1538755171379305}, {4.0823149563929446, -1.9716505566529836}, {-40.524239387703069, 51.084928442746765}},
    {1, 0, {-0.59999999999999998, 0.29999999999999999}, {-1.6256930064551962, -1.2138782508802561}, {0.61010966629001939, 0.92734174568994177}, {8.4295669377933304, -79.162521021692868, -72.41802529909053, 227.9964144477849}, {8.1649099640051386, -7.1056235258589941, -7.1056235258589933, 9.6059376958921749}, {0.20354085503367961, -0.10015910443749557}, {51.286233823777849, -106.71129667966353}},
    {1, 1, {-0.59999999999999998, 0.29999999999999999}, {-1.6256930064551962, -1.2138782508802561}, {0.61010966629001939, 0.92734174568994177}, {-54.858427468590087, -94.276438392076102, -16.688678313932868, 203.57757748611667}, {8.1649099640051386, -7.1056235258589941, -7.1056235258589933, 9.6059376958921749}, {0.82005625062707466, 0.028808935890416967}, {51.286233823777849, -106.71129667966353}},
    {1, 0, {0.20000000000000001, 0.40000000000000002}, {-0.17718840139880299, 1.0706770759293018}, {-0.2074876755660271, -0.98835080978403811}, {119.48173096744141, -41.923774738928458, -39.654628165822764, 4.8283085266380459}, {2.5731958094845186, -1.2861376187452653, -1.2861376187452656, 1.5628171933052348}, {-1.4328503324382231, -3.406427437610581}, {-59.358417206585045, 45.002943257694923}},
    {1, 1, {0.20000000000000001, 0.40000000000000002}, {-0.17718840139880299, 1.0706770759293018}, {-0.2074876755660271, -0.98835080978403811}, {102.39997039719702, -44.459345427733396, -30.155406020411455, -3.9888695218922376}, {2.5731958094845186, -1.2861376187452653, -1.2861376187452656, 1.5628171933052348}, {-1.2253650198299009, -2.1702154697541785}, {-59.358417206585045, 45.002943257694923}},
};
// clang-format on

int failures = 0;

void expectNear(const char *what, double got, double want, double tol)
{
    if (!(std::abs(got - want) <= tol))
    {
        std::printf("FAIL %s: got %.12g, want %.12g (tol %.1e)\n", what, got, want, tol);
        ++failures;
    }
}
} // namespace

int main()
{
    const pa::Vec2 kp = {50.0, 50.0}, kd = {3.0, 3.0};
    double worst_f = 0, worst_J = 0, worst_inv = 0, worst_tau = 0;

    // 1) forward map, Jacobian, inverse and crank torque against the Python reference
    for (const auto &r : kReference)
    {
        const auto side = static_cast<pa::Side>(r.side);
        pa::Vec2 q_m;
        pa::Mat2 J;
        double margin;
        pa::kinematics(side, r.q_s, q_m, J, margin);
        for (int i = 0; i < 2; ++i)
        {
            worst_f = std::max(worst_f, std::abs(q_m[i] - r.q_m[i]));
            for (int j = 0; j < 2; ++j) worst_J = std::max(worst_J, std::abs(J[i][j] - r.J[2 * i + j]));
        }
        expectNear("margin > 0 at reachable pose", margin > 0.0, 1.0, 0.0);

        pa::Vec2 q_s;
        const bool ok = pa::serialAngles(side, r.q_m, {-0.25, 0.0}, q_s);
        expectNear("inverse converged", ok, 1.0, 0.0);
        worst_inv = std::max({worst_inv, std::abs(q_s[0] - r.q_s[0]), std::abs(q_s[1] - r.q_s[1])});

        // crankCommand warm-started slightly off, as with the firmware serial state
        const auto c = pa::crankCommand(side, r.q_m, r.dq_m, {r.q_s[0] + 0.01, r.q_s[1] - 0.01}, r.q_ref, r.dq_ref,
                                        kp, kd, r.tau_ff, 1e9);
        expectNear("crankCommand ok", c.ok, 1.0, 0.0);
        // what the motors apply: kd .* (0 - dq_m) + tau
        for (int i = 0; i < 2; ++i)
        {
            const double applied = c.kd[i] * (0.0 - r.dq_m[i]) + c.tau[i];
            worst_tau = std::max(worst_tau, std::abs(applied - r.tau_m[i]));
        }
        expectNear("crank damping gains are positive", c.kd[0] > 0.0 && c.kd[1] > 0.0, 1.0, 0.0);
    }
    expectNear("max |f - reference| [rad]", worst_f, 0.0, 1e-12);
    expectNear("max |J - reference|", worst_J, 0.0, 1e-9);
    expectNear("max |f^-1(f(q_s)) - q_s| [rad]", worst_inv, 0.0, 1e-9);
    expectNear("max |applied crank torque - J^-T tau_s| [Nm]", worst_tau, 0.0, 1e-8);

    // 2) round trip over the whole serial joint range (pitch -0.87..0.35, roll -0.44..0.44), warm start at the
    //    neighbouring grid point as in the 500 Hz loop, and cold start from the ready pose
    int unreachable = 0, cold_failures = 0;
    double worst_round_trip = 0;
    for (int side = 0; side < 2; ++side)
    {
        for (double p = -0.87; p <= 0.35 + 1e-9; p += 0.02)
        {
            for (double r = -0.44; r <= 0.44 + 1e-9; r += 0.02)
            {
                pa::Vec2 q_m;
                pa::Mat2 J;
                double margin;
                pa::kinematics(static_cast<pa::Side>(side), {p, r}, q_m, J, margin);
                if (margin <= 0.0)
                {
                    ++unreachable;
                    continue;
                }
                pa::Vec2 q_s;
                if (!pa::serialAngles(static_cast<pa::Side>(side), q_m, {p + 0.02, r - 0.02}, q_s))
                {
                    std::printf("FAIL inverse (warm) at side %d q_s (%.2f, %.2f)\n", side, p, r);
                    ++failures;
                    continue;
                }
                worst_round_trip = std::max({worst_round_trip, std::abs(q_s[0] - p), std::abs(q_s[1] - r)});
                if (!pa::serialAngles(static_cast<pa::Side>(side), q_m, {-0.25, 0.0}, q_s) ||
                    std::abs(q_s[0] - p) > 1e-9 || std::abs(q_s[1] - r) > 1e-9)
                {
                    ++cold_failures;
                }
            }
        }
    }
    expectNear("grid round trip [rad]", worst_round_trip, 0.0, 1e-9);

    // 3) torque clipping keeps the sign and the limit
    {
        const auto &r = kReference[1];
        const auto c = pa::crankCommand(static_cast<pa::Side>(r.side), r.q_m, {0.0, 0.0}, r.q_s, {r.q_s[0] + 1.0, r.q_s[1]},
                                        {0.0, 0.0}, kp, kd, {0.0, 0.0}, 5.0);
        expectNear("clipped |tau| <= limit", std::max(std::abs(c.tau[0]), std::abs(c.tau[1])) <= 5.0 + 1e-12, 1.0, 0.0);
        for (int i = 0; i < 2; ++i)
            expectNear("clip keeps sign", (c.tau[i] >= 0) == (c.tau_raw[i] >= 0), 1.0, 0.0);
    }

    // 3b) with crank speed the applied torque, damping included, stays in the limit
    {
        const auto &r = kReference[1];
        for (double v : {-5.0, 5.0})
        {
            const pa::Vec2 dq_m{v, -v};
            const auto c = pa::crankCommand(static_cast<pa::Side>(r.side), r.q_m, dq_m, r.q_s, r.q_s, {0.0, 0.0}, kp,
                                            kd, {0.0, 0.0}, 5.0);
            for (int i = 0; i < 2; ++i)
                expectNear("applied |kd (0 - dq_m) + tau| <= limit",
                           std::abs(c.kd[i] * (0.0 - dq_m[i]) + c.tau[i]) <= 5.0 + 1e-9, 1.0, 0.0);
        }
    }

    // 3c) impedance transfer against impedance_transfer.py, the closure at the measured state, the wire form and the fallback
    {
        double worst_Kp = 0, worst_Kd = 0, worst_qstar = 0, worst_closure = 0, worst_hess = 0;
        int fallbacks = 0;
        for (const auto &r : kImpedanceReference)
        {
            const auto side = static_cast<pa::Side>(r.side);
            const pa::Vec2 q_m = pa::crankAngles(side, r.q_s);
            pa::ImpedanceOptions opt;
            opt.paper_terms = r.literal != 0;
            opt.fallback_eig = -1e9; // compute the law whatever its sign, ok is checked below
            const auto law = pa::impedanceLaw(side, q_m, r.dq_m, {r.q_s[0] + 0.01, r.q_s[1] - 0.01}, r.q_ref, {0.0, 0.0},
                                              kp, kd, {0.0, 0.0}, 1e9, opt);
            expectNear("impedanceLaw estimated q_s", law.estimated, 1.0, 0.0);
            for (int i = 0; i < 2; ++i)
            {
                worst_qstar = std::max(worst_qstar, std::abs(law.q_star[i] - r.q_star[i]));
                for (int j = 0; j < 2; ++j)
                {
                    worst_Kp = std::max(worst_Kp, std::abs(law.Kp[i][j] - r.Kp[2 * i + j]));
                    worst_Kd = std::max(worst_Kd, std::abs(law.Kd[i][j] - r.Kd[2 * i + j]));
                }
            }
            // eq. 32: at the measured state the impedance reproduces tau_m
            const auto m = pa::impedanceMotorCommand(law, q_m, r.dq_m, 1e9);
            for (int i = 0; i < 2; ++i) worst_closure = std::max(worst_closure, std::abs(m.tau_applied[i] - r.tau_m[i]));
            // the fallback fires exactly when Python's Kp_m is not positive definite enough or a diagonal is not positive
            const double min_eig_py = pa::minEigSym({{{r.Kp[0], r.Kp[1]}, {r.Kp[2], r.Kp[3]}}});
            const bool usable = min_eig_py > 1e-3 && r.Kp[0] > 0.0 && r.Kp[3] > 0.0;
            opt.fallback_eig = 1e-3;
            const auto guarded = pa::impedanceLaw(side, q_m, r.dq_m, r.q_s, r.q_ref, {0.0, 0.0}, kp, kd, {0.0, 0.0}, 1e9, opt);
            expectNear("impedance fallback matches the sign of min eig(Kp_m)", guarded.ok == usable, 1.0, 0.0);
            fallbacks += guarded.ok ? 0 : 1;
        }
        // Hessians: central difference of J against a coarse forward difference with a different step
        for (const auto &r : kReference)
        {
            const auto side = static_cast<pa::Side>(r.side);
            pa::Tensor2 H, H2;
            pa::jacobianDerivative(side, r.q_s, H, 1e-6);
            pa::jacobianDerivative(side, r.q_s, H2, 1e-5);
            for (int i = 0; i < 2; ++i)
                for (int m = 0; m < 2; ++m)
                    for (int n = 0; n < 2; ++n) worst_hess = std::max(worst_hess, std::abs(H[i][m][n] - H2[i][m][n]));
        }
        expectNear("max |Kp_m - python| [Nm/rad]", worst_Kp, 0.0, 1e-4);
        expectNear("max |Kd_m - python| [Nm s/rad]", worst_Kd, 0.0, 1e-7);
        expectNear("max |q_m* - python| [rad]", worst_qstar, 0.0, 1e-6);
        expectNear("max |closure - tau_m| [Nm]", worst_closure, 0.0, 1e-6);
        expectNear("Hessian step independence", worst_hess, 0.0, 1e-5);
        std::printf("impedance: Kp %.1e  Kd %.1e  q* %.1e  closure %.1e Nm  hessian %.1e  (%d of %zu cases fall back)\n", worst_Kp,
                    worst_Kd, worst_qstar, worst_closure, worst_hess, fallbacks, sizeof(kImpedanceReference) / sizeof(kImpedanceReference[0]));
    }

    // 3d) wire form: the driver's kp, kd on the diagonal plus tau reproduce Kp_m (q* - q) - Kd_m dq, also after the state moved
    //     and with a cap on kp and kd (the cap's difference goes into tau); tau + driver torque stays in the limit
    {
        const auto &r = kImpedanceReference[0];
        const auto side = static_cast<pa::Side>(r.side);
        pa::ImpedanceOptions opt;
        opt.kp_max = 40.0;
        opt.kd_max = 2.0;
        const pa::Vec2 q_m = pa::crankAngles(side, r.q_s);
        const auto law = pa::impedanceLaw(side, q_m, r.dq_m, r.q_s, r.q_ref, {0.0, 0.0}, kp, kd, {0.0, 0.0}, 1e9, opt);
        expectNear("capped kp", law.kp[0] <= 40.0 + 1e-12 && law.kp[1] <= 40.0 + 1e-12, 1.0, 0.0);
        expectNear("capped kd", law.kd[0] <= 2.0 + 1e-12 && law.kd[1] <= 2.0 + 1e-12, 1.0, 0.0);
        const pa::Vec2 q_later = {q_m[0] + 0.03, q_m[1] - 0.02}, dq_later = {r.dq_m[0] * 0.5, -r.dq_m[1]};
        const auto m = pa::impedanceMotorCommand(law, q_later, dq_later, 1e9);
        for (int i = 0; i < 2; ++i)
        {
            double want = 0.0;
            for (int j = 0; j < 2; ++j) want += law.Kp[i][j] * (law.q_star[j] - q_later[j]) - law.Kd[i][j] * dq_later[j];
            const double got = m.tau[i] + m.kp[i] * (m.q[i] - q_later[i]) + m.kd[i] * (0.0 - dq_later[i]);
            expectNear("wire form equals the impedance at the later state [Nm]", got, want, 1e-9);
        }
        const auto clipped = pa::impedanceMotorCommand(law, q_later, dq_later, 3.0);
        for (int i = 0; i < 2; ++i)
        {
            const double total = clipped.tau[i] + clipped.kp[i] * (clipped.q[i] - q_later[i]) - clipped.kd[i] * dq_later[i];
            expectNear("clipped total torque <= limit", std::abs(total) <= 3.0 + 1e-9, 1.0, 0.0);
        }
    }

    // 4) unreachable crank angles are reported, not silently mapped
    {
        pa::Vec2 q_s;
        expectNear("unreachable cranks rejected", pa::serialAngles(pa::kLeft, {2.5, -2.5}, {-0.25, 0.0}, q_s), 0.0, 0.0);
    }

    std::printf("f %.1e  J %.1e  inverse %.1e  crank torque %.1e Nm  grid round trip %.1e (%d unreachable grid "
                "poses skipped, %d cold-start misses)\n",
                worst_f, worst_J, worst_inv, worst_tau, worst_round_trip, unreachable, cold_failures);
    std::printf(failures == 0 ? "OK\n" : "%d FAILURES\n", failures);
    return failures == 0 ? 0 : 1;
}
