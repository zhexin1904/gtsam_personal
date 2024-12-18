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
         * @param measured pixels in the camera frame
         */
        PlanarProjectionFactorBase(const Point2& measured) : measured_(measured) {}

        /**
         * Predict the projection of the landmark in camera pixels.
         *
         * @param landmark point3 of the target
         * @param wTb "world to body": planar pose2 of vehicle body frame in world frame
         * @param bTc "body to camera": camera pose in body frame, oriented parallel to pose2 zero i.e. +x
         * @param calib camera calibration with distortion
         * @param Hlandmark jacobian
         * @param HwTb jacobian
         * @param HbTc jacobian
         * @param Hcalib jacobian
         */
        Point2 predict(
            const Point3& landmark,
            const Pose2& wTb,
            const Pose3& bTc,
            const Cal3DS2& calib,
            OptionalMatrixType Hlandmark = nullptr, // 2x3 (x, y, z)
            OptionalMatrixType HwTb = nullptr, // 2x3 (x, y, theta)
            OptionalMatrixType HbTc = nullptr, // 2x6 (rx, ry, rz, x, y, theta)
            OptionalMatrixType Hcalib = nullptr // 2x9
        ) const {
#ifndef GTSAM_THROW_CHEIRALITY_EXCEPTION
            try {
#endif
                gtsam::Matrix Hp; // 6x3
                gtsam::Matrix H0; // 6x6
                // this is x-forward z-up
                Pose3 wTc = Pose3::FromPose2(wTb, Hp).compose(bTc, HwTb ? &H0 : nullptr);
                // this is z-forward y-down
                gtsam::Matrix H00; // 6x6
                Pose3 camera_pose = wTc.compose(CAM_COORD, H00);
                PinholeCamera<Cal3DS2> camera = PinholeCamera<Cal3DS2>(camera_pose, calib);
                if (HwTb || HbTc) {
                    // Dpose is for pose3, 2x6 (R,t)
                    gtsam::Matrix Dpose;
                    Point2 result = camera.project(landmark, Dpose, Hlandmark, Hcalib);
                    gtsam::Matrix DposeOffset = Dpose * H00; // 2x6
                    if (HbTc)
                        *HbTc = DposeOffset; // with Eigen this is a deep copy (!)
                    if (HwTb) 
                        *HwTb =  DposeOffset * H0 * Hp;
                    return result;
                } else {
                    return camera.project(landmark, {}, {}, {});
                }
#ifndef GTSAM_THROW_CHEIRALITY_EXCEPTION
            } catch (CheiralityException& e) {
                std::cout << "****** CHIRALITY EXCEPTION ******\n";
                if (Hlandmark) *Hlandmark = Matrix::Zero(2, 3);
                if (HwTb) *HwTb = Matrix::Zero(2, 3);
                if (HbTc) *HbTc = Matrix::Zero(2, 6);
                if (Hcalib) *Hcalib = Matrix::Zero(2, 9);
                // return a large error
                return Matrix::Constant(2, 1, 2.0 * calib.fx());
            }
#endif
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
         * @brief constructor for known landmark, offset, and calibration
         * @param poseKey index of the robot pose2 in the z=0 plane
         * @param landmark point3 in the world
         * @param measured corresponding point2 in the camera frame
         * @param bTc "body to camera": constant camera offset from pose
         * @param calib constant camera calibration
         * @param model stddev of the measurements, ~one pixel?
         */
        PlanarProjectionFactor1(
            Key poseKey,
            const Point3& landmark,
            const Point2& measured,
            const Pose3& bTc,
            const Cal3DS2& calib,
            const SharedNoiseModel& model)
            : PlanarProjectionFactorBase(measured),
            NoiseModelFactorN(model, poseKey),
            landmark_(landmark),
            bTc_(bTc),
            calib_(calib) {
            assert(2 == model->dim());
        }

        /**
         * @param wTb "world to body": estimated pose2
         * @param HwTb jacobian
         */
        Vector evaluateError(
            const Pose2& wTb,
            OptionalMatrixType HwTb = OptionalNone) const override {
            return predict(landmark_, wTb, bTc_, calib_, {}, HwTb, {}, {}) - measured_;
        }

    private:
        Point3 landmark_; // landmark
        Pose3 bTc_; // "body to camera": camera offset to robot pose
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
    class PlanarProjectionFactor2 
    : public PlanarProjectionFactorBase, public NoiseModelFactorN<Pose2, Point3> {
    public:
        typedef NoiseModelFactorN<Pose2, Point3> Base;
        using Base::evaluateError;

        PlanarProjectionFactor2() {}

        ~PlanarProjectionFactor2() override {}

        /// @return a deep copy of this factor
        gtsam::NonlinearFactor::shared_ptr clone() const override {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PlanarProjectionFactor2(*this)));
        }

        /**
         * @brief constructor for variable landmark, known offset and calibration
         * @param poseKey index of the robot pose2 in the z=0 plane
         * @param landmarkKey index of the landmark point3
         * @param measured corresponding point in the camera frame
         * @param bTc "body to camera": constant camera offset from pose
         * @param calib constant camera calibration
         * @param model stddev of the measurements, ~one pixel?
         */
        PlanarProjectionFactor2(
            Key poseKey,
            Key landmarkKey,
            const Point2& measured,
            const Pose3& bTc,
            const Cal3DS2& calib,
            const SharedNoiseModel& model)
            : PlanarProjectionFactorBase(measured),
            NoiseModelFactorN(model, landmarkKey, poseKey),
            bTc_(bTc),
            calib_(calib) {
            assert(2 == model->dim());
        }

        /**
         * @param wTb "world to body": estimated pose2
         * @param landmark estimated landmark
         * @param HwTb jacobian
         * @param Hlandmark jacobian
         */
        Vector evaluateError(
            const Pose2& wTb,
            const Point3& landmark,
            OptionalMatrixType HwTb = OptionalNone,
            OptionalMatrixType Hlandmark = OptionalNone) const override {
            return predict(landmark, wTb, bTc_, calib_, Hlandmark, HwTb, {}, {}) - measured_;
        }

    private:
        Pose3 bTc_; // "body to camera": camera offset to robot pose
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
         * @brief constructor for variable pose, offset, and calibration, known landmark.
         * @param landmark point3 in the world
         * @param measured corresponding point2 in the camera frame
         * @param model stddev of the measurements, ~one pixel?
         * @param poseKey index of the robot pose2 in the z=0 plane
         * @param offsetKey index of camera offset from pose
         * @param calibKey index of camera calibration         */
        PlanarProjectionFactor3(
            Key poseKey,
            Key offsetKey,
            Key calibKey,
            const Point3& landmark,
            const Point2& measured,
            const SharedNoiseModel& model)
            : PlanarProjectionFactorBase(measured),
            NoiseModelFactorN(model, poseKey, offsetKey, calibKey),
            landmark_(landmark) {
            assert(2 == model->dim());
        }

        /**
         * @param wTb "world to body": estimated pose2
         * @param bTc "body to camera": pose3 offset from pose2 +x
         * @param calib calibration
         * @param HwTb pose jacobian
         * @param H2 offset jacobian
         * @param H3 calibration jacobian
         */
        Vector evaluateError(
            const Pose2& wTb,
            const Pose3& bTc,
            const Cal3DS2& calib,
            OptionalMatrixType HwTb = OptionalNone,
            OptionalMatrixType HbTc = OptionalNone,
            OptionalMatrixType Hcalib = OptionalNone) const override {
            return predict(landmark_, wTb, bTc, calib, {}, HwTb, HbTc, Hcalib) - measured_;
        }

    private:
        Point3 landmark_; // landmark
    };

    template<>
    struct traits<PlanarProjectionFactor3> :
        public Testable<PlanarProjectionFactor3> {};

} // namespace gtsam