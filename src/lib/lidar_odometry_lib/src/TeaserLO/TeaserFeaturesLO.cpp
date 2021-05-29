#include "TeaserLO/TeaserFeaturesLO.hpp"
#define NOISE_BOUND 0.05
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10
namespace vlo {
TeaserFeaturesLO::TeaserFeaturesLO(const vlo::Transform3D& initial_pose)
    : initial_pose_(initial_pose){};

void TeaserFeaturesLO::processFrame(IFrame::Ptr lidar_frame) {
    current_lidar_frame_ = lidar_frame;

    if (lo_state_ == VLO_STATE::INIT) {
        current_lidar_frame_->pose_ = initial_pose_;
        pushFrameToHistory(current_lidar_frame_);
        lo_state_ = VLO_STATE::RUNNING;
    } else if (lo_state_ == VLO_STATE::RUNNING) {
        auto [frame_to_frame_transform, trust_result] =
            findFrameToFrameTransformBetween(current_lidar_frame_,
                                             reference_lidar_frame_);
        current_lidar_frame_->pose_.matrix() =
            reference_lidar_frame_->pose_.matrix() *
            frame_to_frame_transform.matrix();
        pushFrameToHistory(current_lidar_frame_);
    }
}
double TeaserFeaturesLO::calculateCrossCheckingError(const IFrame::Ptr & current_frame, const IFrame::Ptr & reference_frame)
{

}

void open3dToTeaserlocal(const PointCloud & input_cloud, teaser::PointCloud & output_cloud)
{
    for (const auto point : input_cloud.points_)
    {
        teaser::PointXYZ current_point;
        current_point.x = point[0];
        current_point.y = point[1];
        current_point.z = point[2];
        output_cloud.push_back(current_point);
    }
}
std::tuple<Transform3D, bool> TeaserFeaturesLO::findFrameToFrameTransformBetween(
    IFrame::Ptr& current_frame, IFrame::Ptr& reference_frame) {
    Transform3D frame_to_frame_transform = Transform3D::Identity();

    teaser::PointCloud reference_point_cloud_teaser, current_point_cloud_teaser;
    current_frame->point_cloud_.EstimateNormals();
    reference_frame->point_cloud_.EstimateNormals();
    current_frame->point_cloud_ = *current_frame->point_cloud_.VoxelDownSample(0.5);
    reference_frame->point_cloud_ = *reference_frame->point_cloud_.VoxelDownSample(0.5);

    auto correspondences_pair = findCorrespondencePointsBasedOnFeatures(current_frame->point_cloud_, reference_frame->point_cloud_);
    std::cout << "coresponece pairs: " << std::endl;
    std::cout << correspondences_pair.size() << std::endl;


    open3dToTeaserlocal(current_frame->point_cloud_, current_point_cloud_teaser);
    open3dToTeaserlocal(reference_frame->point_cloud_, reference_point_cloud_teaser);
    frame_to_frame_transform = useTeaserForTransformCalculations(
        current_point_cloud_teaser, reference_point_cloud_teaser,
        correspondences_pair);

    return {frame_to_frame_transform, true};
}

Transform3D TeaserFeaturesLO::useTeaserForTransformCalculations(
    const teaser::PointCloud& current_point_cloud,
    const teaser::PointCloud& reference_point_cloud,
    const std::vector<std::pair<int, int>>& pairs) {
    // Run TEASER++ registration
    // Prepare solver parameters

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // Solve with TEASER++
    std::cout << " Calculations ... " << std::endl;
    std::cout << current_point_cloud.size() << std::endl;
    std::cout << reference_point_cloud.size() << std::endl;
    teaser::RobustRegistrationSolver solver(params);

    solver.solve(current_point_cloud, reference_point_cloud, pairs);
    auto solution = solver.getSolution();

    std::cout << "=====================================" << std::endl;
    std::cout << "          TEASER++ Results           " << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Failed?" << std::endl;
    std::cout << solution.valid << std::endl;
    std::cout << "Expected rotation: " << std::endl;
    std::cout << "Estimated rotation: " << std::endl;
    std::cout << solution.rotation << std::endl;
    std::cout << 180.0 / M_PI * solution.rotation.eulerAngles(0, 1, 2)
              << std::endl;
    std::cout << "Estimated translation: " << std::endl;
    std::cout << solution.translation << std::endl;
    std::cout << "Estimated scale: " << std::endl;
    std::cout << solution.scale << std::endl;

    Transform3D result;
    result.translation() = solution.translation;
    result.matrix().block<3, 3>(0, 0) = solution.rotation;

    return result;
}

std::vector<std::pair<int, int>>
TeaserFeaturesLO::findCorrespondencePointsBasedOnFeatures(
    const PointCloud& current_point_cloud,
    const PointCloud& reference_point_cloud) {
    std::vector<std::pair<int, int>> correspondences_pair;
    auto current_features = *open3d::pipelines::registration::ComputeFPFHFeature(
        current_point_cloud/*,
        open3d::geometry::KDTreeSearchParamHybrid(0.25, 100)*/);

    auto reference_features =
        *open3d::pipelines::registration::ComputeFPFHFeature(
            reference_point_cloud/*,
            open3d::geometry::KDTreeSearchParamHybrid(0.25, 100)*/);
    std::cout << "current features: " << current_features.Num() << std::endl;
    std::cout << "referenfe features: " << reference_features.Dimension() << std::endl;

    std::vector<std::reference_wrapper<
        const open3d::pipelines::registration::CorrespondenceChecker>>
        correspondence_checker;
    auto correspondence_checker_edge_length =
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
           2.0);
//    auto correspondence_checker_distance =
//        open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
//            0.075);
//    auto correspondence_checker_normal =
//        open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
//            0.52359878);

//    correspondence_checker.push_back(correspondence_checker_edge_length);
//    correspondence_checker.push_back(correspondence_checker_distance);
//    correspondence_checker.push_back(correspondence_checker_normal);

    auto results =
        open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
            current_point_cloud, reference_point_cloud, current_features,
            reference_features, 2.0,
            open3d::pipelines::registration::TransformationEstimationPointToPoint(
                false),
            4/*, correspondence_checker,
            open3d::pipelines::registration::RANSACConvergenceCriteria(4000000,
                                                                       1000)*/);
    std::cout << "What;s going on: " << results.correspondence_set_.size() << std::endl;
    for (int i = 0; i < results.correspondence_set_.size(); i++) {
        std::pair<int, int> pair_correspondence{results.correspondence_set_[i][0],
                                                results.correspondence_set_[i][1]};
        correspondences_pair.emplace_back(pair_correspondence);
    }
    return correspondences_pair;
}

void TeaserFeaturesLO::hintForICP(const Transform3D& guess_for_icp) {
    guess_for_icp_ = guess_for_icp;
}

};  // namespace vlo
