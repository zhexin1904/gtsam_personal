/**
 * ProjectionFactor, but with pose2 (robot on the floor)
 *
 * This factor is useful for high-school robotics competitions,
 * which run robots on the floor, with use fixed maps and fiducial
 * markers.
 *
 * @see https://www.firstinspires.org/
 *
 * @file PlanarProjectionFactor.h
 * @brief for planar smoothing
 * @date Dec 2, 2024
 * @author joel@truher.org
 */
#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>


namespace gtsam {

    /**
     * @class PlanarProjectionFactorBase
     * @brief Camera projection for robot on the floor.  Measurement is camera pixels.
     */
    class PlanarProjectionFactorBase {
    protected:
        PlanarProjectionFactorBase() {}

        /**
         * @param measured corresponding point in the camera frame
         * @param poseKey index of the robot pose in the z=0 plane
         */
        PlanarProjectionFactorBase(const Point2& measured) : measured_(measured) {}

        /**
         * @param landmark point3
         * @param pose
         * @param offset oriented parallel to pose2 zero i.e. +x
         * @param calib
         * @param H1 landmark jacobian
         * @param H2 pose jacobian
         * @param H3 offset jacobian
         * @param H4 calib jacobian
         */
        Point2 h(
            const Point3& landmark,
            const Pose2& pose,
            const Pose3& offset,
            const Cal3DS2& calib,
            OptionalMatrixType H1, // 2x3 (x, y, z)
            OptionalMatrixType H2, // 2x3 (x, y, theta)
            OptionalMatrixType H3, // 2x6 (rx, ry, rz, x, y, theta)
            OptionalMatrixType H4  // 2x9
        ) const {
            try {
                // this is x-forward z-up
                gtsam::Matrix H0; // 6x6
                Pose3 offset_pose = Pose3(pose).compose(offset, H0);
                // this is z-forward y-down
                gtsam::Matrix H00; // 6x6
                Pose3 camera_pose = offset_pose.compose(CAM_COORD, H00);
                PinholeCamera<Cal3DS2> camera = PinholeCamera<Cal3DS2>(camera_pose, calib);
                if (H2 || H3) {
                    // Dpose is for pose3, 2x6 (R,t)
                    gtsam::Matrix Dpose;
                    Point2 result = camera.project(landmark, Dpose, H1, H4);
                    gtsam::Matrix DposeOffset = Dpose * H00; // 2x6
                    if (H3)
                        *H3 = DposeOffset; // with Eigen this is a deep copy (!)
                    if (H2) {
                        gtsam::Matrix DposeOffsetFwd = DposeOffset * H0;
                        *H2 = Matrix::Zero(2, 3);
                        (*H2)(0, 0) = DposeOffsetFwd(0, 3); // du/dx
                        (*H2)(1, 0) = DposeOffsetFwd(1, 3); // dv/dx
                        (*H2)(0, 1) = DposeOffsetFwd(0, 4); // du/dy
                        (*H2)(1, 1) = DposeOffsetFwd(1, 4); // dv/dy
                        (*H2)(0, 2) = DposeOffsetFwd(0, 2); // du/dyaw
                        (*H2)(1, 2) = DposeOffsetFwd(1, 2); // dv/dyaw
                    }
                    return result;
                } else {
                    return camera.project(landmark, {}, {}, {});
                }
            } catch (CheiralityException& e) {
                std::cout << "****** CHIRALITY EXCEPTION ******\n";
                if (H1) *H1 = Matrix::Zero(2, 3);
                if (H2) *H2 = Matrix::Zero(2, 3);
                if (H3) *H3 = Matrix::Zero(2, 6);
                if (H4) *H4 = Matrix::Zero(2, 9);
                // return a large error
                return Matrix::Constant(2, 1, 2.0 * calib.fx());
            }
        }

        Point2 measured_; // pixel measurement

    private:
        static const Pose3 CAM_COORD;
    };

    // camera "zero" is facing +z; this turns it to face +x
    const Pose3 PlanarProjectionFactorBase::CAM_COORD = Pose3(
        Rot3(0, 0, 1,//
            -1, 0, 0, //
            0, -1, 0),
        Vector3(0, 0, 0)
    );

    /**
     * @class PlanarProjectionFactor1
     * @brief One variable: the pose.
     * Landmark, camera offset, camera calibration are constant.
     * This is intended for localization within a known map.
     */
    class PlanarProjectionFactor1
        : public PlanarProjectionFactorBase, public NoiseModelFactorN<Pose2> {
    public:
        typedef NoiseModelFactorN<Pose2> Base;
        using Base::evaluateError;
        PlanarProjectionFactor1() {}

        ~PlanarProjectionFactor1() override {}

        /// @return a deep copy of this factor
        gtsam::NonlinearFactor::shared_ptr clone() const override {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PlanarProjectionFactor1(*this)));
        }


        /**
         * @param landmark point3 in the world
         * @param measured corresponding point2 in the camera frame
         * @param offset constant camera offset from pose
         * @param calib constant camera calibration
         * @param model stddev of the measurements, ~one pixel?
         * @param poseKey index of the robot pose2 in the z=0 plane
         */
        PlanarProjectionFactor1(
            const Point3& landmark,
            const Point2& measured,
            const Pose3& offset,
            const Cal3DS2& calib,
            const SharedNoiseModel& model,
            Key poseKey)
            : PlanarProjectionFactorBase(measured),
            NoiseModelFactorN(model, poseKey),
            landmark_(landmark),
            offset_(offset),
            calib_(calib) {
            assert(2 == model->dim());
        }

        /**
         * @param pose estimated pose2
         * @param H1 pose jacobian
         */
        Vector evaluateError(
            const Pose2& pose,
            OptionalMatrixType H1 = OptionalNone) const override {
            return h(landmark_, pose, offset_, calib_, {}, H1, {}, {}) - measured_;
        }

    private:
        Point3 landmark_; // landmark
        Pose3 offset_; // camera offset to robot pose
        Cal3DS2 calib_; // camera calibration
    };

    template<>
    struct traits<PlanarProjectionFactor1> :
        public Testable<PlanarProjectionFactor1> {};

    /**
     * @class PlanarProjectionFactor2
     * @brief Two unknowns: the pose and the landmark.
     * Camera offset and calibration are constant.
     * This is similar to GeneralSFMFactor, used for SLAM.
    */
    class PlanarProjectionFactor2 : public PlanarProjectionFactorBase, public NoiseModelFactorN<Point3, Pose2> {
    public:
        typedef NoiseModelFactorN<Point3, Pose2> Base;
        using Base::evaluateError;

        PlanarProjectionFactor2() {}

        ~PlanarProjectionFactor2() override {}

        /// @return a deep copy of this factor
        gtsam::NonlinearFactor::shared_ptr clone() const override {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PlanarProjectionFactor2(*this)));
        }

        /**
         * @param measured corresponding point in the camera frame
         * @param offset constant camera offset from pose
         * @param calib constant camera calibration
         * @param model stddev of the measurements, ~one pixel?
         * @param poseKey index of the robot pose2 in the z=0 plane
         * @param landmarkKey index of the landmark point3
         */
        PlanarProjectionFactor2(
            const Point2& measured,
            const Pose3& offset,
            const Cal3DS2& calib,
            const SharedNoiseModel& model,
            Key landmarkKey,
            Key poseKey)
            : PlanarProjectionFactorBase(measured),
            NoiseModelFactorN(model, landmarkKey, poseKey),
            offset_(offset),
            calib_(calib) {
            assert(2 == model->dim());
        }

        /**
         * @param landmark estimated landmark
         * @param pose estimated pose2
         * @param H1 landmark jacobian
         * @param H2 pose jacobian
         */
        Vector evaluateError(
            const Point3& landmark,
            const Pose2& pose,
            OptionalMatrixType H1 = OptionalNone,
            OptionalMatrixType H2 = OptionalNone) const override {
            return h(landmark, pose, offset_, calib_, H1, H2, {}, {}) - measured_;
        }

    private:
        Pose3 offset_; // camera offset to robot pose
        Cal3DS2 calib_; // camera calibration
    };

    template<>
    struct traits<PlanarProjectionFactor2> :
        public Testable<PlanarProjectionFactor2> {};

    /**
     * @class PlanarProjectionFactor3
     * @brief Three unknowns: the pose, the camera offset, and the camera calibration.
     * Landmark is constant.
     * This is intended to be used for camera calibration.
    */
    class PlanarProjectionFactor3 : public PlanarProjectionFactorBase, public NoiseModelFactorN<Pose2, Pose3, Cal3DS2> {
    public:
        typedef NoiseModelFactorN<Pose2, Pose3, Cal3DS2> Base;
        using Base::evaluateError;

        PlanarProjectionFactor3() {}

        ~PlanarProjectionFactor3() override {}

        /// @return a deep copy of this factor
        gtsam::NonlinearFactor::shared_ptr clone() const override {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PlanarProjectionFactor3(*this)));
        }

        /**
         * @param landmark point3 in the world
         * @param measured corresponding point2 in the camera frame
         * @param model stddev of the measurements, ~one pixel?
         * @param poseKey index of the robot pose2 in the z=0 plane
         * @param offsetKey index of camera offset from pose
         * @param calibKey index of camera calibration         */
        PlanarProjectionFactor3(
            const Point3& landmark,
            const Point2& measured,
            const SharedNoiseModel& model,
            Key poseKey,
            Key offsetKey,
            Key calibKey)
            : PlanarProjectionFactorBase(measured),
            NoiseModelFactorN(model, poseKey, offsetKey, calibKey),
            landmark_(landmark) {
            assert(2 == model->dim());
        }

        /**
         * @param pose estimated pose2
         * @param offset pose3 offset from pose2 +x
         * @param calib calibration
         * @param H1 pose jacobian
         * @param H2 offset jacobian
         * @param H3 calibration jacobian
         */
        Vector evaluateError(
            const Pose2& pose,
            const Pose3& offset,
            const Cal3DS2& calib,
            OptionalMatrixType H1 = OptionalNone,
            OptionalMatrixType H2 = OptionalNone,
            OptionalMatrixType H3 = OptionalNone) const override {
            return h(landmark_, pose, offset, calib, {}, H1, H2, H3) - measured_;
        }

    private:
        Point3 landmark_; // landmark
    };

    template<>
    struct traits<PlanarProjectionFactor3> :
        public Testable<PlanarProjectionFactor3> {};

} // namespace gtsam