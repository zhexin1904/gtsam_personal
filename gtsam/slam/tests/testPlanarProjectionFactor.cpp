/**
 * @file testPlanarProjectionFactor.cpp
 * @date Dec 3, 2024
 * @author joel@truher.org
 * @brief unit tests for PlanarProjectionFactor
 */

#include <random>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PlanarProjectionFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::C;
using symbol_shorthand::K;

TEST(PlanarProjectionFactor1, error1) {
    // landmark is on the camera bore (which faces +x)
    Point3 landmark(1, 0, 0);
    // so pixel measurement is (cx, cy)
    Point2 measured(200, 200);
    Pose3 offset;
    Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    Values values;
    Pose2 pose(0, 0, 0);
    values.insert(X(0), pose);

    PlanarProjectionFactor1 factor(landmark, measured, offset, calib, model, X(0));

    CHECK_EQUAL(2, factor.dim());
    CHECK(factor.active(values));
    std::vector<Matrix> actualHs(1);
    gtsam::Vector actual = factor.unwhitenedError(values, actualHs);

    CHECK(assert_equal(Vector2(0, 0), actual));

    const Matrix& H1Actual = actualHs.at(0);
    CHECK_EQUAL(2, H1Actual.rows());
    CHECK_EQUAL(3, H1Actual.cols());
    const Matrix23 H1Expected = (Matrix23() << //
        0, 200, 200,//
        0, 0, 0).finished();
    CHECK(assert_equal(H1Expected, H1Actual, 1e-6));
}

TEST(PlanarProjectionFactor1, error2) {
    // landmark is in the upper left corner
    Point3 landmark(1, 1, 1);
    // upper left corner in pixels
    Point2 measured(0, 0);
    Pose3 offset;
    Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    PlanarProjectionFactor1 factor(landmark, measured, offset, calib, model, X(0));
    Values values;
    Pose2 pose(0, 0, 0);

    values.insert(X(0), pose);

    CHECK_EQUAL(2, factor.dim());
    CHECK(factor.active(values));
    std::vector<Matrix> actualHs(1);
    gtsam::Vector actual = factor.unwhitenedError(values, actualHs);

    CHECK(assert_equal(Vector2(0, 0), actual));

    const Matrix& H1Actual = actualHs.at(0);
    CHECK_EQUAL(2, H1Actual.rows());
    CHECK_EQUAL(3, H1Actual.cols());
    Matrix23 H1Expected = (Matrix23() << //
        -200, 200, 400, //
        -200, 0, 200).finished();
    CHECK(assert_equal(H1Expected, H1Actual, 1e-6));
}

TEST(PlanarProjectionFactor1, error3) {
    // landmark is in the upper left corner
    Point3 landmark(1, 1, 1);
    // upper left corner in pixels
    Point2 measured(0, 0);
    Pose3 offset;
    // distortion
    Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    PlanarProjectionFactor1 factor(landmark, measured, offset, calib, model, X(0));
    Values values;
    Pose2 pose(0, 0, 0);

    values.insert(X(0), pose);

    CHECK_EQUAL(2, factor.dim());
    CHECK(factor.active(values));
    std::vector<Matrix> actualHs(1);
    gtsam::Vector actual = factor.unwhitenedError(values, actualHs);

    CHECK(assert_equal(Vector2(0, 0), actual));

    const Matrix& H1Actual = actualHs.at(0);
    CHECK_EQUAL(2, H1Actual.rows());
    CHECK_EQUAL(3, H1Actual.cols());
    Matrix23 H1Expected = (Matrix23() << //
        -360, 280, 640, //
        -360, 80, 440).finished();
    CHECK(assert_equal(H1Expected, H1Actual, 1e-6));
}

TEST(PlanarProjectionFactor1, jacobian) {
    // test many jacobians with many randoms

    std::default_random_engine g;
    std::uniform_real_distribution<double> s(-0.3, 0.3);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));

    for (int i = 0; i < 1000; ++i) {
        Point3 landmark(2 + s(g), s(g), s(g));
        Point2 measured(200 + 100*s(g), 200 + 100*s(g));
        Pose3 offset(Rot3::Ypr(s(g),s(g),s(g)), Point3(s(g),s(g),s(g)));
        Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);

        PlanarProjectionFactor1 factor(landmark, measured, offset, calib, model, X(0));

        Pose2 pose(s(g), s(g), s(g));

        // actual H
        Matrix H1;
        factor.evaluateError(pose, H1);

        Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
            [&factor](const Pose2& p) {
                return factor.evaluateError(p);},
                pose);
        CHECK(assert_equal(expectedH1, H1, 1e-6));
    }
}



TEST(PlanarProjectionFactor3, error1) {
    // landmark is on the camera bore (facing +x)
    Point3 landmark(1, 0, 0);
    // so px is (cx, cy)
    Point2 measured(200, 200);
    // offset is identity
    Pose3 offset;
    Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    Values values;
    Pose2 pose(0, 0, 0);
    values.insert(X(0), pose);
    values.insert(C(0), offset);
    values.insert(K(0), calib);

    PlanarProjectionFactor3 factor(landmark, measured, model, X(0), C(0), K(0));

    CHECK_EQUAL(2, factor.dim());
    CHECK(factor.active(values));
    std::vector<Matrix> actualHs(3);

    gtsam::Vector actual = factor.unwhitenedError(values, actualHs);
    CHECK(assert_equal(Vector2(0, 0), actual));

    const Matrix& H1Actual = actualHs.at(0);
    const Matrix& H2Actual = actualHs.at(1);
    const Matrix& H3Actual = actualHs.at(2);

    CHECK_EQUAL(2, H1Actual.rows());
    CHECK_EQUAL(3, H1Actual.cols());
    CHECK_EQUAL(2, H2Actual.rows());
    CHECK_EQUAL(6, H2Actual.cols());
    CHECK_EQUAL(2, H3Actual.rows());
    CHECK_EQUAL(9, H3Actual.cols());

    // du/dx etc for the pose2d
    Matrix23 H1Expected = (Matrix23() <<//
        0, 200, 200,//
        0, 0, 0).finished();
    CHECK(assert_equal(H1Expected, H1Actual, 1e-6));

    // du/dx for the pose3d offset
    // note this is (roll, pitch, yaw, x, y, z)
    Matrix26 H2Expected = (Matrix26() <<//
        0, 0, 200, 0, 200, 0,//
        0, -200, 0, 0, 0, 200).finished();
    CHECK(assert_equal(H2Expected, H2Actual, 1e-6));

    // du wrt calibration
    // on-bore, f doesn't matter
    // but c does
    Matrix29 H3Expected = (Matrix29() <<//
        0, 0, 0, 1, 0, 0, 0, 0, 0,//
        0, 0, 0, 0, 1, 0, 0, 0, 0).finished();
    CHECK(assert_equal(H3Expected, H3Actual, 1e-6));
}

TEST(PlanarProjectionFactor3, error2) {
    Point3 landmark(1, 1, 1);
    Point2 measured(0, 0);
    Pose3 offset;
    Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);

    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    PlanarProjectionFactor3 factor(landmark, measured, model, X(0), C(0), K(0));
    Values values;
    Pose2 pose(0, 0, 0);

    values.insert(X(0), pose);
    values.insert(C(0), offset);
    values.insert(K(0), calib);


    CHECK_EQUAL(2, model->dim());
    CHECK_EQUAL(2, factor.dim());
    CHECK(factor.active(values));
    std::vector<Matrix> actualHs(3);
    gtsam::Vector actual = factor.unwhitenedError(values, actualHs);

    CHECK(assert_equal(Vector2(0, 0), actual));

    const Matrix& H1Actual = actualHs.at(0);
    const Matrix& H2Actual = actualHs.at(1);
    const Matrix& H3Actual = actualHs.at(2);

    CHECK_EQUAL(2, H1Actual.rows());
    CHECK_EQUAL(3, H1Actual.cols());
    CHECK_EQUAL(2, H2Actual.rows());
    CHECK_EQUAL(6, H2Actual.cols());
    CHECK_EQUAL(2, H3Actual.rows());
    CHECK_EQUAL(9, H3Actual.cols());

    Matrix23 H1Expected = (Matrix23() <<//
        -200, 200, 400,//
        -200, 0, 200).finished();
    CHECK(assert_equal(H1Expected, H1Actual, 1e-6));

    // du/dx for the pose3d offset
    // note this is (roll, pitch, yaw, x, y, z)
    Matrix26 H2Expected = (Matrix26() <<//
        -200, -200, 400, -200, 200, 0,//
        200, -400, 200, -200, 0, 200).finished();
    CHECK(assert_equal(H2Expected, H2Actual, 1e-6));

    Matrix29 H3Expected = (Matrix29() <<//
        -1, 0, -1, 1, 0, -400, -800, 400, 800,//
        0, -1, 0, 0, 1, -400, -800, 800, 400).finished();

    CHECK(assert_equal(H3Expected, H3Actual, 1e-6));
}

TEST(PlanarProjectionFactor3, error3) {
    Point3 landmark(1, 1, 1);
    Point2 measured(0, 0);
    Pose3 offset;
    Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);

    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    PlanarProjectionFactor3 factor(landmark, measured, model, X(0), C(0), K(0));
    Values values;
    Pose2 pose(0, 0, 0);

    values.insert(X(0), pose);
    values.insert(C(0), offset);
    values.insert(K(0), calib);


    CHECK_EQUAL(2, model->dim());
    CHECK_EQUAL(2, factor.dim());
    CHECK(factor.active(values));
    std::vector<Matrix> actualHs(3);
    gtsam::Vector actual = factor.unwhitenedError(values, actualHs);

    CHECK(assert_equal(Vector2(0, 0), actual));

    const Matrix& H1Actual = actualHs.at(0);
    const Matrix& H2Actual = actualHs.at(1);
    const Matrix& H3Actual = actualHs.at(2);

    CHECK_EQUAL(2, H1Actual.rows());
    CHECK_EQUAL(3, H1Actual.cols());
    CHECK_EQUAL(2, H2Actual.rows());
    CHECK_EQUAL(6, H2Actual.cols());
    CHECK_EQUAL(2, H3Actual.rows());
    CHECK_EQUAL(9, H3Actual.cols());

    Matrix23 H1Expected = (Matrix23() <<//
        -360, 280, 640,//
        -360, 80, 440).finished();
    CHECK(assert_equal(H1Expected, H1Actual, 1e-6));

    // du/dx for the pose3d offset
    // note this is (roll, pitch, yaw, x, y, z)
    Matrix26 H2Expected = (Matrix26() <<//
        -200, -440, 640, -360, 280, 80,//
        200, -640, 440, -360, 80, 280).finished();
    CHECK(assert_equal(H2Expected, H2Actual, 1e-6));

    Matrix29 H3Expected = (Matrix29() <<//
        -1, 0, -1, 1, 0, -400, -800, 400, 800,//
        0, -1, 0, 0, 1, -400, -800, 800, 400).finished();

    CHECK(assert_equal(H3Expected, H3Actual, 1e-6));
}


TEST(PlanarProjectionFactor3, jacobian) {
    // test many jacobians with many randoms

    std::default_random_engine g;
    std::uniform_real_distribution<double> s(-0.3, 0.3);    
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));

    for (int i = 0; i < 1000; ++i) {
        Point3 landmark(2 + s(g), s(g), s(g));
        Point2 measured(200 + 100*s(g), 200 + 100*s(g));
        Pose3 offset(Rot3::Ypr(s(g),s(g),s(g)), Point3(s(g),s(g),s(g)));
        Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);

        PlanarProjectionFactor3 factor(landmark, measured, model, X(0), C(0), K(0));

        Pose2 pose(s(g), s(g), s(g));

        // actual H
        Matrix H1, H2, H3;
        factor.evaluateError(pose, offset, calib, H1, H2, H3);

        Matrix expectedH1 = numericalDerivative31<Vector, Pose2, Pose3, Cal3DS2>(
            [&factor](const Pose2& p, const Pose3& o, const Cal3DS2& c) {
                return factor.evaluateError(p, o, c);},
                pose, offset, calib);
        Matrix expectedH2 = numericalDerivative32<Vector, Pose2, Pose3, Cal3DS2>(
            [&factor](const Pose2& p, const Pose3& o, const Cal3DS2& c) {
                return factor.evaluateError(p, o, c);},
                pose, offset, calib);
        Matrix expectedH3 = numericalDerivative33<Vector, Pose2, Pose3, Cal3DS2>(
            [&factor](const Pose2& p, const Pose3& o, const Cal3DS2& c) {
                return factor.evaluateError(p, o, c);},
                pose, offset, calib);
        CHECK(assert_equal(expectedH1, H1, 1e-6));
        CHECK(assert_equal(expectedH2, H2, 1e-6));
        CHECK(assert_equal(expectedH3, H3, 1e-6));
    }
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
