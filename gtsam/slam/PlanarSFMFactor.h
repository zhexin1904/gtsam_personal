/**
 * Similar to GeneralSFMFactor, but:
 *
 * * 2d pose variable (robot on the floor)
 * * camera offset variable
 * * constant landmarks
 * * batched input
 * * numeric differentiation
 *
 * This factor is useful to find camera calibration and placement, in
 * a sort of "autocalibrate" mode.  Once a satisfactory solution is
 * found, the PlanarProjectionFactor should be used for localization.
 *
 * The python version of this factor uses batches, to save on calls
 * across the C++/python boundary, but here the only extra cost
 * is instantiating the camera, so there's no batch.
 *
 * @see https://www.firstinspires.org/
 * @see PlanarProjectionFactor.h
 *
 * @file PlanarSFMFactor.h
 * @brief for planar smoothing with unknown calibration
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
     * @class PlanarSFMFactor
     * @brief Camera calibration for robot on the floor.
     */
    class PlanarSFMFactor : public NoiseModelFactorN<Pose2, Pose3, Cal3DS2> {
    private:
        typedef NoiseModelFactorN<Pose2, Pose3, Cal3DS2> Base;   
        static const Pose3 CAM_COORD;

    protected:

        Point3 landmark_; // landmark
        Point2 measured_; // pixel measurement

    public:
        // Provide access to the Matrix& version of evaluateError:
        using Base::evaluateError;

        PlanarSFMFactor() {}
        /**
         * @param landmarks point in the world
         * @param measured corresponding point in the camera frame
         * @param model stddev of the measurements, ~one pixel?
         * @param poseKey index of the robot pose2 in the z=0 plane
         * @param offsetKey index of the 3d camera offset from the robot pose
         * @param calibKey index of the camera calibration
         */
        PlanarSFMFactor(
            const Point3& landmark,
            const Point2& measured,
            const SharedNoiseModel& model,
            Key poseKey,
            Key offsetKey,
            Key calibKey)
            : NoiseModelFactorN(model, poseKey, offsetKey, calibKey),
            landmark_(landmark), measured_(measured)
        {
            assert(2 == model->dim());
        }

        ~PlanarSFMFactor() override {}

        /// @return a deep copy of this factor
        gtsam::NonlinearFactor::shared_ptr clone() const override {
            return std::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new PlanarSFMFactor(*this)));
        }

        Point2 h2(const Pose2& pose,
            const Pose3& offset,
            const Cal3DS2& calib,
            OptionalMatrixType H1,
            OptionalMatrixType H2,
            OptionalMatrixType H3
            ) const {
            // this is x-forward z-up
            gtsam::Matrix H0;
            Pose3 offset_pose = Pose3(pose).compose(offset, H0);
            // this is z-forward y-down
            gtsam::Matrix H00;
            Pose3 camera_pose = offset_pose.compose(CAM_COORD, H00);
            PinholeCamera<Cal3DS2> camera = PinholeCamera<Cal3DS2>(camera_pose, calib);
            if (H1 || H2) {
                gtsam::Matrix Dpose;
                Point2 result = camera.project(landmark_, Dpose, {}, H3);
                gtsam::Matrix DposeOffset = Dpose * H00;
                if (H2)
                    *H2 = DposeOffset; // a deep copy
                if (H1) {
                    gtsam::Matrix DposeOffsetFwd = DposeOffset * H0;
                    *H1 = Matrix::Zero(2,3);
                    (*H1)(0,0) = DposeOffsetFwd(0,3); // du/dx
                    (*H1)(1,0) = DposeOffsetFwd(1,3); // dv/dx
                    (*H1)(0,1) = DposeOffsetFwd(0,4); // du/dy
                    (*H1)(1,1) = DposeOffsetFwd(1,4); // dv/dy
                    (*H1)(0,2) = DposeOffsetFwd(0,2); // du/dyaw
                    (*H1)(1,2) = DposeOffsetFwd(1,2); // dv/dyaw
                }
                return result;
            } else {
                return camera.project(landmark_, {}, {}, {});
            }    
        }

        Vector evaluateError(
            const Pose2& pose,
            const Pose3& offset,
            const Cal3DS2& calib,
            OptionalMatrixType H1 = OptionalNone,
            OptionalMatrixType H2 = OptionalNone,
            OptionalMatrixType H3 = OptionalNone
        ) const override {
            try {
                return h2(pose,offset,calib,H1,H2,H3) - measured_;
            }
            catch (CheiralityException& e) {
                std::cout << "****** CHIRALITY EXCEPTION ******\n";
                std::cout << "landmark " << landmark_ << "\n";
                std::cout << "pose " << pose << "\n";
                std::cout << "offset " << offset << "\n";
                std::cout << "calib " << calib << "\n";
                if (H1) *H1 = Matrix::Zero(2, 3);
                if (H2) *H2 = Matrix::Zero(2, 6);
                if (H3) *H3 = Matrix::Zero(2, 9);
                // return a large error
                return Matrix::Constant(2, 1, 2.0 * calib.fx());
            }
        }
    };

    // camera "zero" is facing +z; this turns it to face +x
    const Pose3 PlanarSFMFactor::CAM_COORD = Pose3(
        Rot3(0, 0, 1,//
            -1, 0, 0, //
            0, -1, 0),
        Vector3(0, 0, 0)
    );

    template<>
    struct traits<PlanarSFMFactor> :
        public Testable<PlanarSFMFactor > {
    };

} // namespace gtsam
