//  points_current.push_back(current->getGoodKeyPoints()[100].pt);
//  points_current.push_back(current->getGoodKeyPoints()[150].pt);
//  points_current.push_back(current->getGoodKeyPoints()[200].pt);
//  points_current.push_back(current->getGoodKeyPoints()[250].pt);
//  cv::computeCorrespondEpilines(points_reference, 1, fundamental_matrix, epilines1);


//  cv::Mat out_img;
//  KeyPoints temp_keypoints;
//  temp_keypoints.push_back(reference->getGoodKeyPoints()[0]);
//temp_keypoints.push_back(reference->getGoodKeyPoints()[10]);
//  temp_keypoints.push_back(reference->getGoodKeyPoints()[20]);
//  temp_keypoints.push_back(reference->getGoodKeyPoints()[30]);
//  temp_keypoints.push_back(reference->getGoodKeyPoints()[40]);
//  temp_keypoints.push_back(reference->getGoodKeyPoints()[50]);

//  KeyPoints temp_keypoints2;
//  temp_keypoints2.push_back(current->getGoodKeyPoints()[0]);
// temp_keypoints2.push_back(current->getGoodKeyPoints()[10]);
// temp_keypoints2.push_back(current->getGoodKeyPoints()[20]);
//  temp_keypoints2.push_back(current->getGoodKeyPoints()[30]);
//  temp_keypoints2.push_back(current->getGoodKeyPoints()[40]);
//  temp_keypoints2.push_back(current->getGoodKeyPoints()[50]);

//  cv::Mat img_current = current->image_;
//  cv::Mat img_reference = reference->image_;

//  for (size_t i = 0; i < points_current.size() ; i ++)
//  {

//      auto img = current->image_;

//     cv::line(img_current, cv::Point(0,-epilines1[i][2]/epilines1[i][1]),cv::Point(img.cols,-(epilines1[i][2]+epilines1[i][0]*img.cols)/epilines1[i][1]) , cv::Scalar(0,255.0,0));
//       cv::circle(img_current, points_current[i],8,cv::Scalar(0,255,0.0));


//  }

//  cv::imshow("img_current", img_current);
//  cv::imshow("img_reference", img_reference);
//cv::waitKey(0);



//  cv::drawKeypoints(reference->image_, temp_keypoints, out_img,
//                    cv::Scalar(0, 0, 255));
//  cv::imshow("out_img", out_img);
//  cv::Mat out_img2;
//  cv::drawKeypoints(current->image_, temp_keypoints2, out_img2,
//                    cv::Scalar(0, 0, 255));
//  cv::imshow("out_img2", out_img2);
//  cv::waitKey(0);
//  exit(0);

//  std::cout << "essential matrix: " << essential_matrix << std::endl;

//  Eigen::Matrix3d b = Eigen::Matrix3d::Zero();

//  Eigen::Matrix<double, 4,3> P1_inv;
//  P1_inv << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
//  Eigen::Matrix<double,3,4> P2;

//  P2.matrix().block<3, 1>(0, 3) =
//  transform_from_current_reference.translation(); P2.matrix().block<3, 3>(0,
//  0) = transform_from_current_reference.rotation();

//  auto P = P2 * P1_inv;
//  auto K = current->sensor_->getSensorMatrix();
//  std::cout << " K : " << K << std::endl;
//  std::cout << K.inv() << std::endl;
//  Eigen::Matrix<float,3,3> K_eigen, K_eigen_inv;
//  cv::cv2eigen(K, K_eigen);
//  cv::cv2eigen(K.inv(), K_eigen_inv);
//  auto chain =  K_eigen * P.cast<float>() * K_eigen_inv ;
//  std::cout << "chain: " << chain << std::endl;

// std::vector<cv::KeyPoint> keypoints_projected;
// std::cout << "inverse calculation" << std::endl;

//  for (auto keypoint : reference->getGoodKeyPoints()) {
//    Eigen::Vector3d x1(keypoint.pt.x, keypoint.pt.y, 1.0);

//    Eigen::Vector3f new_point = chain.inverse() * x1.cast<float>();
//    new_point = new_point / new_point(2);
//    std::cout << "essential: " << new_point.transpose() *
//    essential_matrix.cast<float>() * new_point << std::endl; std::cout << "
//    New point: " << std::endl; std::cout << new_point << std::endl;
//    cv::KeyPoint keypoint_transformed;
//    keypoint_transformed.pt.x = new_point(0);
//    keypoint_transformed.pt.y = new_point(1);
//    if (isInImage(current->image_, keypoint.pt))
//    {
//        keypoints_projected.push_back(keypoint_transformed);
//    }

//  }
//  std::cout << "here" << std::endl;
//  cv::Mat out_img;
//  cv::drawKeypoints(reference->image_,keypoints_projected,out_img);
//  cv::imshow("current image and projected keypoints", out_img);

//  cv::Mat out_img2;
//  cv::drawKeypoints(reference->image_, reference->getGoodKeyPoints(),
//  out_img2); cv::imshow("reference img and reference keypoints", out_img2);

//  cv::waitKey(0);

//  for (auto keypoint : reference->getGoodKeyPoints()) {
//    Vec3 point3d = Eigen::Vector3d(keypoint.pt.x, keypoint.pt.y, 0.0);
//    std::cout << point3d << std::endl;
//    std::cout << "lol " << point3d.homogeneous() << std::endl;
//    std::cout << transform_from_current_reference.matrix() << std::endl;
//    Eigen::Matrix4d mat =
//        transform_from_current_reference.matrix();  //*
//        point3d.homogeneous();
//    std::cout << "mat: " << mat << std::endl;
//    auto lol = mat * point3d.homogeneous();
//    std::cout << "lol " << lol << std::endl;
//    auto normalized_point = lol / lol(3);
//    std::cout << "noramlized point: " << normalized_point << std::endl;
//    point3d(0) = normalized_point(0);
//    point3d(1) = normalized_point(1);
//    point3d(2) = normalized_point(2);
//    std::cout << "point: " << point3d << std::endl;
//    cv::Point2d point2d = projectPointToImagePlane(point3d,
//    current->sensor_->getSensorMatrix()); std::cout << "projected point: "
//    << point2d << std::endl; cv::KeyPoint keypoint_transformed;
//    keypoint_transformed.pt = point2d;

//    keypoints_projected.push_back(keypoint_transformed);

//  }
