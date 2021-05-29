#include "VOwithScaleVLO/VOwithScaleVLO.hpp"



namespace vlo {
VOwithScaleVLO::VOwithScaleVLO(
    const cv::Ptr<cv::Feature2D> &descriptor,
    const cv::Ptr<cv::DescriptorMatcher> &matcher,
    const Transform3D &initial_pose = Transform3D::Identity())
    : IVisualOdometer(initial_pose),
    feature_detector_(FeatureDetector(descriptor)),
    feature_matcher_(FeatureMatcher(matcher)){};

void VOwithScaleVLO::processFrame(IFrame::Ptr &frame) {
    current_frame_ = frame;
    if (vo_state_ == VLO_STATE::INIT) {
        current_frame_->pose_ = initial_pose_;
        addKeyFrame(current_frame_);
        vo_state_ = VLO_STATE::RUNNING;
    } else if (vo_state_ == VLO_STATE::RUNNING) {
        auto [frame_to_frame_transform, trust_result] =
            findFrameToFrameTransformBetween(current_frame_, reference_frame_);
        std::cout << "Local transformation before ackermann " << frame_to_frame_transform.translation()
                  << std::endl;

        if (!sanity_checker_.trustResult(current_frame_, this->history_frames_)) {
            std::cout << "Don't trust the result..." << std::endl;
            sanity_checker_.checkAckermann(
                frame_to_frame_transform,
                current_frame_->time_stamp_ - reference_frame_->time_stamp_);
        }
        std::cout << "Local transformation after ackerman: " << frame_to_frame_transform.translation()
                  << std::endl;

        if (trust_result) {
            //      if (scale_ > 0.0 and frame_to_frame_transform.translation()[2] <
            //      0.0) {
            //        if (debug_) {
            //          std::cout << "wrong direction " << std::endl;
            //          cv::waitKey(0);
            //        }
            //      }

            Transform3D current_pose =
                reference_frame_->pose_ * frame_to_frame_transform;

            Eigen::Quaterniond q_current(current_pose.rotation().matrix());
            q_current.normalize();
            current_pose.matrix().block<3, 3>(0, 0) = q_current.toRotationMatrix();
            current_frame_->pose_ = current_pose;

        } else {
            if (debug_) {
                std::cout << "Constant Velocity model!" << std::endl;
                cv::waitKey(0);
            }
            auto last_transform = getFrameToFrameTransformation();

            last_transform.matrix().block<3, 1>(0, 3) =
                scale_ * last_transform.translation().normalized();
            last_transform.matrix().block<3, 3>(0, 0) = Mat33::Identity();

            Transform3D current_pose = reference_frame_->pose_ * last_transform;
            Eigen::Quaterniond q_current(current_pose.rotation().matrix());
            q_current.normalize();
            current_pose.matrix().block<3, 3>(0, 0) = q_current.toRotationMatrix();
            current_frame_->pose_ = current_pose;
        }

        addKeyFrame(current_frame_);
    }
}

std::tuple<Transform3D, bool>
VOwithScaleVLO::imageToimageTransform5point(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
    cv::Mat mask;
    cv::Mat triangulated_points;
    auto essential_matrix = cv::findEssentialMat(
        current_frame->getGoodPoints(), reference_frame->getGoodPoints(),
        current_frame->sensor_->getSensorMatrix(), cv::RANSAC, 0.999, 0.05, mask);
    cv::Mat t;
    cv::Mat rotation = cv::Mat_<double>::eye(3, 3);
    cv::recoverPose(essential_matrix, current_frame->getGoodPoints(),
                    reference_frame->getGoodPoints(),
                    current_frame->sensor_->getSensorMatrix(), rotation, t, mask);



    if (std::abs(cv::determinant(rotation)) - 1.0 > 1e-07) {
        std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
        exit(EXIT_FAILURE);
    }

    Mat33 d_rotation_eigen;
    Vec3 d_translation_eigen;
    cv2eigen(rotation, d_rotation_eigen);
    cv2eigen(t, d_translation_eigen);
    d_translation_eigen.normalize();
    Eigen::Quaterniond q(d_rotation_eigen);
    q.normalize();
    d_rotation_eigen = q.toRotationMatrix();

    Transform3D frame_to_frame_transform = Transform3D::Identity();
    frame_to_frame_transform.translation() = scale_ * d_translation_eigen;
    frame_to_frame_transform.matrix().block<3, 3>(0, 0) = d_rotation_eigen;
    bool trust_result = false;
    if (verbose_) {
        std::cout << "VO det R local" << d_rotation_eigen.determinant()
                  << std::endl;
        std::cout << "VO scale local: " << scale_ << std::endl;
        std::cout << "VO translation local " << std::endl;
        std::cout << d_translation_eigen << std::endl;
        std::cout << "VO rotation local " << std::endl;
        std::cout << d_rotation_eigen << std::endl;
        std::cout << "VO euler angels local " << std::endl;
        std::cout << d_rotation_eigen.eulerAngles(0, 1, 2) * 180.0 / M_PI
                  << std::endl;
    }
    std::cout << "Points that are reliable for transformation matching: "
              << cv::countNonZero(mask) << std::endl;
    if (cv::countNonZero(mask) > 5) {
        trust_result = true;
    }

    return {frame_to_frame_transform, trust_result};
}

std::tuple<Transform3D, bool>
VOwithScaleVLO::findFrameToFrameTransformBetween(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
    findCurrentImageToReferenceImageMatches(current_frame, reference_frame);
    auto [frame_to_frame_transform, trust_result] =
        imageToimageTransform5point(current_frame, reference_frame);
    return {frame_to_frame_transform, trust_result};
}

void VOwithScaleVLO::calculateScale(const Vec3 &d_translation) {
    auto length = d_translation.norm();
    scale_ = length;
}

void VOwithScaleVLO::findCurrentImageToReferenceImageMatches(
    IFrame::Ptr &current_frame, IFrame::Ptr &reference_frame) {
    IndexMatches good_matches;
    if (verbose_) {
        std::cout << "Descriptor sizes current vs reference: " << std::endl;
        std::cout << current_frame->descriptors_.size() << std::endl;
        std::cout << reference_frame->descriptors_.size() << std::endl;
    }
    feature_matcher_.match(current_frame->descriptors_,
                           reference_frame->descriptors_, good_matches);
    Indexes good_current_indexes, good_last_indexes;
    for (size_t i = 0; i < good_matches.size(); i++) {
        good_current_indexes.push_back(good_matches[i].queryIdx);
        good_last_indexes.push_back(good_matches[i].trainIdx);
    }
    if (verbose_) {
        std::cout << "Good matches between frames" << std::endl;
        std::cout << good_matches.size() << std::endl;
        //    std::ofstream myfile;
        //    myfile.open("number_matches7.txt", std::ios_base::app);
        //    myfile << good_matches.size() << "\n";
        //    myfile.close();
    }

    current_frame->updateGoodIndexes(good_current_indexes);
    reference_frame->updateGoodIndexes(good_last_indexes);
    if (debug_) {
        cv::Mat img_out;
        cv::drawMatches(current_frame->image_, current_frame->key_points_,
                        reference_frame->image_, reference_frame->key_points_,
                        good_matches, img_out);
        cv::imshow("match", img_out);
        cv::waitKey(1);
    }
}
double VOwithScaleVLO::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{

}
void VOwithScaleVLO::addKeyFrame(IFrame::Ptr frame) {
    pushFrameToHistory(frame);
    reference_frame_ = frame;
}
};  // namespace vlo
