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

TEST(PlanarProjectionFactor, error1) {
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

    PlanarProjectionFactor factor(landmark, measured, offset, calib, model, X(0));

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

TEST(PlanarProjectionFactor, error2) {
    // landmark is in the upper left corner
    Point3 landmark(1, 1, 1);
    // upper left corner in pixels
    Point2 measured(0, 0);
    Pose3 offset;
    Cal3DS2 calib(200, 200, 0, 200, 200, 0, 0);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    PlanarProjectionFactor factor(landmark, measured, offset, calib, model, X(0));
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

TEST(PlanarProjectionFactor, error3) {
    // landmark is in the upper left corner
    Point3 landmark(1, 1, 1);
    // upper left corner in pixels
    Point2 measured(0, 0);
    Pose3 offset;
    // distortion
    Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));
    PlanarProjectionFactor factor(landmark, measured, offset, calib, model, X(0));
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

TEST(PlanarProjectionFactor, jacobian) {
    // test many jacobians with many randoms

    std::default_random_engine g;
    std::uniform_real_distribution<double> s(-0.3, 0.3);
    SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(1, 1));

    for (int i = 0; i < 1000; ++i) {
        Point3 landmark(2 + s(g), s(g), s(g));
        Point2 measured(200 + 100*s(g), 200 + 100*s(g));
        Pose3 offset(Rot3::Ypr(s(g),s(g),s(g)), Point3(s(g),s(g),s(g)));
        Cal3DS2 calib(200, 200, 0, 200, 200, -0.2, 0.1);

        PlanarProjectionFactor factor(landmark, measured, offset, calib, model, X(0));

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

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

