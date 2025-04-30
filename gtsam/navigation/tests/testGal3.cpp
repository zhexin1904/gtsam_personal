/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testGal3.cpp
 * @brief   Unit tests for Gal3 class, mirroring custom_gal3_tests.py
 * @author  Based on Python tests by User and NavState tests
 * @date    April 29, 2025 // Updated Date
 */

#include <gtsam/navigation/Gal3.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/testLie.h> // Include for Lie group concepts if needed later
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h> // Point3 is already included via Gal3.h usually

#include <CppUnitLite/TestHarness.h>
#include <vector>
#include <functional> // For std::bind if using numerical derivatives

using namespace std;
using namespace gtsam;

// Define tolerance matching Python TOL
static const double kTol = 1e-8;
GTSAM_CONCEPT_TESTABLE_INST(gtsam::Gal3)
GTSAM_CONCEPT_LIE_INST(gtsam::Gal3)

// Instantiate Testable concept for Gal3 to allow assert_equal(Gal3, Gal3)
// Instantiate Lie group concepts if you plan to use CHECK_LIE_GROUP_DERIVATIVES etc.
// GTSAM_CONCEPT_LIE_INST(Gal3)

// ========================================================================
// Test Cases Section (mirroring Python's TestGal3 class)
// ========================================================================

/* ************************************************************************* */
TEST(Gal3, Identity) {
    // Hardcoded expected data
    const Matrix3 expected_R_mat = Matrix3::Identity();
    const Vector3 expected_r_vec = Vector3::Zero();
    const Vector3 expected_v_vec = Vector3::Zero();
    const double expected_t_val = 0.0;

    Gal3 custom_ident = Gal3::Identity();

    // Check components individually
    EXPECT(assert_equal(expected_R_mat, custom_ident.rotation().matrix(), kTol));
    // Ensure translation() returns Point3, then convert if needed or compare Point3
    EXPECT(assert_equal(Point3(expected_r_vec), custom_ident.translation(), kTol));
    EXPECT(assert_equal(expected_v_vec, custom_ident.velocity(), kTol));
    EXPECT_DOUBLES_EQUAL(expected_t_val, custom_ident.time(), kTol);

    // Alternative: Check whole object if Testable concept works as expected
    EXPECT(assert_equal(Gal3(Rot3(expected_R_mat), Point3(expected_r_vec), expected_v_vec, expected_t_val), custom_ident, kTol));
}

/* ************************************************************************* */
TEST(Gal3, HatVee) {
    // Hardcoded test cases for Hat/Vee (from Python script)
    // Case 1
    const Vector10 tau_vec_1 = (Vector10() <<
        -0.9919387548344049, 0.41796965721894275, -0.08567669832855362, // rho
         -0.24728318780816563, 0.42470896104182426, 0.37654216726012074, // nu
         -0.4665439537974297, -0.46391731412948783, -0.46638346398428376, // theta
         -0.2703091399101363 // t_tan
    ).finished();
    const Matrix5 expected_xi_matrix_1 = (Matrix5() <<
         0.0, 0.46638346398428376, -0.46391731412948783, -0.24728318780816563, -0.9919387548344049,
        -0.46638346398428376, 0.0, 0.4665439537974297, 0.42470896104182426, 0.41796965721894275,
         0.46391731412948783, -0.4665439537974297, 0.0, 0.37654216726012074, -0.08567669832855362,
         0.0, 0.0, 0.0, 0.0, -0.2703091399101363,
         0.0, 0.0, 0.0, 0.0, 0.0
    ).finished();

    // Test Hat operation
    Matrix5 custom_Xi_1 = Gal3::Hat(tau_vec_1);
    EXPECT(assert_equal(expected_xi_matrix_1, custom_Xi_1, kTol));

    // Test Vee operation (using the expected Xi matrix as input)
    Vector10 custom_tau_rec_1 = Gal3::Vee(expected_xi_matrix_1);
    EXPECT(assert_equal(tau_vec_1, custom_tau_rec_1, kTol));

    // Test Vee(Hat(tau)) round trip
    Vector10 custom_tau_rec_roundtrip_1 = Gal3::Vee(custom_Xi_1);
    EXPECT(assert_equal(tau_vec_1, custom_tau_rec_roundtrip_1, kTol));

    // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
TEST(Gal3, Expmap) {
    // Hardcoded test case for Expmap (from Python script)
    // Case 1
    const Vector10 tau_vec_1 = (Vector10() <<
        -0.6659680127970163, 0.0801034296770802, -0.005425197747099379,  // rho
        -0.24823309993679712, -0.3282613511681929, -0.3614305580886979,  // nu
         0.3267045270397072, -0.21318895514136532, -0.1810111529904679,  // theta
        -0.11137002094819903 // t_tan
    ).finished();
    const Matrix3 expected_R_mat_1 = (Matrix3() <<
         0.961491754653074, 0.14119138670272793, -0.23579354964696073,
        -0.20977429976081094, 0.9313179826319476, -0.297727322203087,
         0.17756223949368974, 0.33572579219851445, 0.925072885538575
    ).finished();
    const Vector3 expected_r_vec_1 = (Vector3() << -0.637321673031978, 0.16116104619552254, -0.03248254605908951).finished();
    const Vector3 expected_v_vec_1 = (Vector3() << -0.22904001980446076, -0.23988916442951638, -0.4308710747620977).finished();
    const double expected_t_val_1 = -0.11137002094819903;

    Gal3 custom_exp_1 = Gal3::Expmap(tau_vec_1);

    // Check components individually
    EXPECT(assert_equal(expected_R_mat_1, custom_exp_1.rotation().matrix(), kTol));
    EXPECT(assert_equal(Point3(expected_r_vec_1), custom_exp_1.translation(), kTol));
    EXPECT(assert_equal(expected_v_vec_1, custom_exp_1.velocity(), kTol));
    EXPECT_DOUBLES_EQUAL(expected_t_val_1, custom_exp_1.time(), kTol);

    // Check derivatives (optional, but good practice like in testNavState)
    // Matrix9 aH;
    // Gal3::Expmap(tau_vec_1, aH);
    // std::function<Gal3(const Vector9&)> expmap_func = [](const Vector9& v){ return Gal3::Expmap(v); };
    // Matrix expectedH = numericalDerivative11(expmap_func, tau_vec_1);
    // EXPECT(assert_equal(expectedH, aH, 1e-6)); // Use appropriate tolerance

    // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
TEST(Gal3, Logmap) {
    // --- Part 1: Test Logmap(GroupElement) ---
    // Hardcoded test case 1 (from Python script)
    const Matrix3 input_R_mat_1 = (Matrix3() <<
        -0.8479395778141634, 0.26601628670932354, -0.4585126035145831,
         0.5159883846729487, 0.612401478276553, -0.5989327310201821,
         0.1214679351060988, -0.744445944720015, -0.6565407650184298
    ).finished();
    // Use Point3/Velocity3 directly for construction
    const Point3 input_r_vec_1(0.6584021866593519, -0.3393257406257678, -0.5420636579124554);
    const Velocity3 input_v_vec_1(0.1772802663861217, 0.3146080790621266, 0.7173535084539808);
    const double input_t_val_1 = -0.12088016100268817;
    const Gal3 custom_g_1(Rot3(input_R_mat_1), input_r_vec_1, input_v_vec_1, input_t_val_1);

    const Vector10 expected_log_tau_1 = (Vector10() <<
        -0.6366686897004876, -0.2565186503803428, -1.1108185946230884, // rho
         1.122213550821757, -0.21828427331226408, 0.03100839182220047, // nu
        -0.6312616056853186, -2.516056355068653, 1.0844223965979727,  // theta
        -0.12088016100268817 // t_tan
    ).finished();

    Vector10 custom_log_tau_1 = Gal3::Logmap(custom_g_1);
    EXPECT(assert_equal(expected_log_tau_1, custom_log_tau_1, kTol*1e3));
    // Add more test cases here if they exist in the Python script


    // --- Part 2: Test Log(Exp(tau)) round trip ---
    // Using data from the Expmap test case
    const Vector10 tau_vec_orig_rt1 = (Vector10() <<
        -0.6659680127970163, 0.0801034296770802, -0.005425197747099379,
        -0.24823309993679712, -0.3282613511681929, -0.3614305580886979,
         0.3267045270397072, -0.21318895514136532, -0.1810111529904679,
        -0.11137002094819903
    ).finished();
    Gal3 g_exp_rt1 = Gal3::Expmap(tau_vec_orig_rt1);
    Vector10 tau_log_exp_rt1 = Gal3::Logmap(g_exp_rt1);
    // Use slightly larger tolerance for round trip
    EXPECT(assert_equal(tau_vec_orig_rt1, tau_log_exp_rt1, kTol * 10));


    // --- Part 3: Test Exp(Log(g)) round trip ---
    // Using data from the first Logmap test case
    Gal3 custom_g_orig_rt2 = custom_g_1; // Reuse from Part 1
    Vector10 tau_log_rt2 = Gal3::Logmap(custom_g_orig_rt2);
    Gal3 g_exp_log_rt2 = Gal3::Expmap(tau_log_rt2);
    // Compare the reconstructed g against the original using assert_equal for Gal3
    EXPECT(assert_equal(custom_g_orig_rt2, g_exp_log_rt2, kTol * 10));
}

/* ************************************************************************* */
TEST(Gal3, Compose) {
    // Hardcoded test case 1 (from Python script)
    const Matrix3 g1_R_mat_1 = (Matrix3() <<
        -0.5427153955878299, 0.8391431164114453, 0.03603927817303032,
         0.8040805715986894, 0.5314810596281534, -0.2664250694549225,
        -0.24272295682417533, -0.11561450357036887, -0.963181630220753
    ).finished();
    const Point3 g1_r_vec_1(0.9978490360071179, -0.5634861893781862, 0.025864788808796835);
    const Velocity3 g1_v_vec_1(0.04857438013356852, -0.012834026018545996, 0.945550047767139);
    const double g1_t_val_1 = -0.41496643117394594;
    const Gal3 c1_1(Rot3(g1_R_mat_1), g1_r_vec_1, g1_v_vec_1, g1_t_val_1);

    const Matrix3 g2_R_mat_1 = (Matrix3() <<
        -0.3264538540162394, 0.24276278793202133, -0.9135064914894779,
         0.9430076454867458, 0.1496317101600385, -0.2972321178273404,
         0.06453264097716288, -0.9584761760784951, -0.27777501352435885
    ).finished();
    const Point3 g2_r_vec_1(-0.1995427558196442, 0.7830589040103644, -0.433370507989717);
    const Velocity3 g2_v_vec_1(0.8986541507293722, 0.051990700444202176, -0.8278883042875157);
    const double g2_t_val_1 = -0.6155723080111539;
    const Gal3 c2_1(Rot3(g2_R_mat_1), g2_r_vec_1, g2_v_vec_1, g2_t_val_1);

    // Expected result of composition
    const Matrix3 expected_R_mat_1 = (Matrix3() <<
         0.9708156167565788, -0.04073147244077803, 0.23634294026763253,
         0.22150238776832248, 0.5300893429357028, -0.8184998355032984,
        -0.09194417042137741, 0.8469629482207595, 0.5236411308011658
    ).finished();
    const Point3 expected_r_vec_1(1.7177230471349891, -0.18439262777314758, -0.18087448280827323);
    const Velocity3 expected_v_vec_1(-0.4253479212754224, 0.9579585887030762, 1.5188219826819997);
    const double expected_t_val_1 = -1.0305387391850998;
    const Gal3 expected_comp_1(Rot3(expected_R_mat_1), expected_r_vec_1, expected_v_vec_1, expected_t_val_1);

    Gal3 custom_comp_1 = c1_1 * c2_1; // Or c1_1.compose(c2_1)

    // Compare the whole Gal3 object
    EXPECT(assert_equal(expected_comp_1, custom_comp_1, kTol));

    // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
TEST(Gal3, Inverse) {
    // Hardcoded identity for comparison
    Gal3 expected_identity = Gal3::Identity();

    // Hardcoded test case 1 (from Python script)
    const Matrix3 g_R_mat_1 = (Matrix3() <<
         0.6680516673568877, 0.2740542884848495, -0.6918101016209183,
         0.6729369985913887, -0.6193062871756463, 0.4044941514923666,
        -0.31758898858193396, -0.7357676057205693, -0.5981498680963873
    ).finished();
    const Point3 g_r_vec_1(0.06321286832132045, -0.9214393132931736, -0.12472480681013542);
    const Velocity3 g_v_vec_1(0.4770686298036335, 0.2799576331302327, -0.29190264050471715);
    const double g_t_val_1 = 0.3757227805330059;
    const Gal3 custom_g_1(Rot3(g_R_mat_1), g_r_vec_1, g_v_vec_1, g_t_val_1);

    // Expected inverse
    const Matrix3 expected_inv_R_mat_1 = (Matrix3() <<
         0.6680516673568877, 0.6729369985913887, -0.31758898858193396,
         0.2740542884848495, -0.6193062871756463, -0.7357676057205693,
        -0.6918101016209183, 0.4044941514923666, -0.5981498680963873
    ).finished();
    const Point3 expected_inv_r_vec_1(0.7635904739613719, -0.6150700906051861, 0.32598918251792036);
    const Velocity3 expected_inv_v_vec_1(-0.5998054073176801, -0.17213568846657853, 0.042198146082895516);
    const double expected_inv_t_val_1 = -0.3757227805330059;
    const Gal3 expected_inv_1(Rot3(expected_inv_R_mat_1), expected_inv_r_vec_1, expected_inv_v_vec_1, expected_inv_t_val_1);

    // Test inverse calculation
    Gal3 custom_inv_1 = custom_g_1.inverse();
    EXPECT(assert_equal(expected_inv_1, custom_inv_1, kTol));

    // Check g * g.inverse() == Identity
    Gal3 ident_c_1 = custom_g_1 * custom_inv_1;
    EXPECT(assert_equal(expected_identity, ident_c_1, kTol));

    // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
TEST(Gal3, Between) {
     // Hardcoded test case 1 (from Python script)
    const Matrix3 g1_R_mat_1 = (Matrix3() <<
        -0.2577418495238488, 0.7767168385303765, 0.5747000015202739,
        -0.6694062876067332, -0.572463082520484, 0.47347781496466923,
         0.6967527259484512, -0.26267274676776586, 0.6674868290752107
    ).finished();
    const Point3 g1_r_vec_1(0.680375434309419, -0.21123414636181392, 0.5661984475172117);
    const Velocity3 g1_v_vec_1(0.536459189623808, -0.44445057839362445, 0.10793991159086103);
    const double g1_t_val_1 = -0.0452058962756795;
    const Gal3 c1_1(Rot3(g1_R_mat_1), g1_r_vec_1, g1_v_vec_1, g1_t_val_1);

    const Matrix3 g2_R_mat_1 = (Matrix3() <<
         0.1981112115076329, -0.12884463237051902, 0.9716743325745948,
         0.9776658305457298, -0.04497580389201028, -0.20529661674659028,
         0.0701532013404006, 0.9906443548385938, 0.11705678352030657
    ).finished();
    const Point3 g2_r_vec_1(0.9044594503494257, 0.8323901360074013, 0.2714234559198019);
    const Velocity3 g2_v_vec_1(-0.5142264587405261, -0.7255368464279626, 0.6083535084539808);
    const double g2_t_val_1 = -0.6866418214918308;
    const Gal3 c2_1(Rot3(g2_R_mat_1), g2_r_vec_1, g2_v_vec_1, g2_t_val_1);

    // Expected result of c1.between(c2) which is c1.inverse() * c2
    const Matrix3 expected_R_mat_1 = (Matrix3() <<
        -0.6566377699430239, 0.753549894443112, -0.0314546605295386,
        -0.4242286152401611, -0.3345440819370167, 0.8414929228771536,
         0.6235839326792096, 0.5659000033801861, 0.5393517081447288
    ).finished();
    const Point3 expected_r_vec_1(-0.8113603292280276, 0.06632931940009146, 0.535144598419476);
    const Velocity3 expected_v_vec_1(0.8076311151757332, -0.7866187376416414, -0.4028976707214998);
    const double expected_t_val_1 = -0.6414359252161513;
    const Gal3 expected_btw_1(Rot3(expected_R_mat_1), expected_r_vec_1, expected_v_vec_1, expected_t_val_1);


    Gal3 custom_btw_1 = c1_1.between(c2_1);
    EXPECT(assert_equal(expected_btw_1, custom_btw_1, kTol));

     // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
TEST(Gal3, MatrixComponents) {
    // Hardcoded test case 1 (from Python script)
    const Matrix3 source_R_mat_1 = (Matrix3() <<
         0.43788117516687186, -0.12083239518241493, -0.8908757538001356,
         0.4981128609611659, 0.8575347951217139, 0.12852102124027118,
         0.7484274541861499, -0.5000336063006573, 0.43568651389548174
    ).finished();
    const Point3 source_r_vec_1(0.3684370505476542, 0.8219440615838134, -0.03501868668711683);
    const Velocity3 source_v_vec_1(0.7621243390078305, 0.282161192634218, -0.13609316346053646);
    const double source_t_val_1 = 0.23919296788014144;
    const Gal3 c1(Rot3(source_R_mat_1), source_r_vec_1, source_v_vec_1, source_t_val_1);

    // Expected matrix representation
    Matrix5 expected_Mc;
    expected_Mc << source_R_mat_1, source_v_vec_1, source_r_vec_1,
                   Vector3::Zero().transpose(), 1.0, source_t_val_1,
                   Vector4::Zero().transpose(), 1.0;

    Matrix5 Mc = c1.matrix();

    // Compare the whole matrix
    EXPECT(assert_equal(expected_Mc, Mc, kTol));

    // Optional: Individual component checks (redundant if the above passes)
    // EXPECT(assert_equal(source_R_mat_1, Mc.block<3,3>(0,0), kTol));
    // EXPECT(assert_equal(source_v_vec_1, Mc.block<3,1>(0,3), kTol));
    // EXPECT(assert_equal(source_r_vec_1.vector(), Mc.block<3,1>(0,4), kTol)); // .vector() for Point3
    // EXPECT_DOUBLES_EQUAL(source_t_val_1, Mc(3,4), kTol);
    // EXPECT_DOUBLES_EQUAL(1.0, Mc(3,3), kTol);
    // EXPECT_DOUBLES_EQUAL(1.0, Mc(4,4), kTol);
    // EXPECT(assert_equal(Vector3::Zero(), Mc.block<1,3>(3,0).transpose(), kTol));
    // EXPECT(assert_equal(Vector4::Zero(), Mc.block<1,4>(4,0).transpose(), kTol));
}

/* ************************************************************************* */
TEST(Gal3, Associativity) {
    // Hardcoded test case 1 (from Python script)
    const Vector10 tau1_1 = (Vector10() <<
         0.14491869060866264, -0.21431172810692092, -0.4956042914756127,
        -0.13411549788475238, 0.44534636811395767, -0.33281648500962074,
        -0.012095339359589743, -0.20104157502639808, 0.08197507996416392,
        -0.006285789459736368
    ).finished();
    const Vector10 tau2_1 = (Vector10() <<
         0.29551108535517384, 0.2938672197508704, 0.0788811297200055,
        -0.07107463852205165, 0.22379088709954872, -0.26508231911443875,
        -0.11625916165500054, 0.04661407377867886, 0.1788274027858556,
        -0.015243024791797033
    ).finished();
     const Vector10 tau3_1 = (Vector10() <<
         0.37646834040234084, 0.3782542871960659, -0.6525520272351378,
         0.005426723127791683, 0.09951505587485733, 0.3813252061414707,
        -0.09289643299325814, -0.0017149201262141199, -0.08507973168896384,
        -0.1109049754985476
    ).finished();

    Gal3 g1_1 = Gal3::Expmap(tau1_1);
    Gal3 g2_1 = Gal3::Expmap(tau2_1);
    Gal3 g3_1 = Gal3::Expmap(tau3_1);

    Gal3 left_assoc_1 = (g1_1 * g2_1) * g3_1;
    Gal3 right_assoc_1 = g1_1 * (g2_1 * g3_1);

    // Compare the results directly using assert_equal for Gal3
    EXPECT(assert_equal(left_assoc_1, right_assoc_1, kTol * 10)); // Slightly larger tol for composed operations

    // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
TEST(Gal3, IdentityProperties) {
    Gal3 custom_identity = Gal3::Identity();

    // Hardcoded test case 1 (from Python script)
    const Matrix3 g_R_mat_1 = (Matrix3() <<
        -0.5204974727334908, 0.7067813015326174, 0.4791060140322894,
         0.773189425449982, 0.15205374379417114, 0.6156766776243058,
         0.3622989004266723, 0.6908978584436601, -0.6256194178153267
    ).finished();
    const Point3 g_r_vec_1(-0.8716573584227159, -0.9599539022706234, -0.08459652545144625);
    const Velocity3 g_v_vec_1(0.7018395735425127, -0.4666685012479632, 0.07952068144433233);
    const double g_t_val_1 = -0.24958604725524136;
    const Gal3 custom_g_1(Rot3(g_R_mat_1), g_r_vec_1, g_v_vec_1, g_t_val_1);

    // Test g * g.inverse() == identity
    Gal3 g_inv_1 = custom_g_1.inverse();
    Gal3 g_times_inv_1 = custom_g_1 * g_inv_1;
    EXPECT(assert_equal(custom_identity, g_times_inv_1, kTol * 10)); // Slightly larger tol

    // Test identity * g == g
    Gal3 id_times_g_1 = custom_identity * custom_g_1;
    EXPECT(assert_equal(custom_g_1, id_times_g_1, kTol));

    // Test g * identity == g
    Gal3 g_times_id_1 = custom_g_1 * custom_identity;
     EXPECT(assert_equal(custom_g_1, g_times_id_1, kTol));

    // Add more test cases here if they exist in the Python script
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
