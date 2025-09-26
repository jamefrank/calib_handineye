#include "cmdline.h"
#include "ba_handineye.h"
#include "utils.h"

#include <spdlog/spdlog.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include <iostream>
using namespace std;



int main(int argc, char *argv[])
{
  spdlog::info("Welcome to use calib_handineye calib tool!");

  cmdline::parser parser;
  parser.add<string>("input", 'i', "Input directory containing images + arm_pose.txt + config.yaml", true, "");
  parser.add<string>("extension", 'e', "File extension of images", false, ".png");
  parser.add<string>("output", 'o', "Output directory containing calib results", true, "");

  parser.add<string>("board-type", 't', "board type", false, "charuco", cmdline::oneof<string>("charuco", "chess"));
  parser.add<int>("board-width", 'w', "Number of inner corners on the chessboard pattern in x direction", false, 3);
  parser.add<int>("board-height", 'h', "Number of inner corners on the chessboard pattern in y direction", false, 2);
  parser.add<double>("square-size", 's', "Size of one square in mm", false, 36.0);
  parser.add<double>("marker-size", 'm', "Size of one square in mm", false, 27.0);

  parser.add("verbose", '\0', "verbose when calib");

  parser.parse_check(argc, argv);
  bool verbose = parser.exist("verbose");

  //
  cv::Mat R_cam2gripper = cv::Mat(3,3,CV_64FC1);				//相机与机械臂末端坐标系的旋转矩阵与平移矩阵
  cv::Mat T_cam2gripper = cv::Mat(3,1,CV_64FC1);
  cv::Mat Homo_cam2gripper = cv::Mat(4,4,CV_64FC1);

  //
  spdlog::info("loading data...");
  std::string inputDir = parser.get<std::string>("input");
  std::string outputDir = parser.get<std::string>("output");
  std::string arm_pose_file = inputDir + "/arm_pose.txt";
  cv::Mat ee_poses = calib_eyeinhand::utils::load_arm_pose(arm_pose_file);
  spdlog::info("ee_pose nums: {}", ee_poses.rows);

  std::string config_file = inputDir + "/config.yaml";
  YAML::Node config = YAML::LoadFile(config_file);
  cv::Mat K, D;
  calib_eyeinhand::utils::parse_K(config, "K", K);
  calib_eyeinhand::utils::parse_D(config, "D", D);
  calib_eyeinhand::utils::log_cvmat(K, "K");
  calib_eyeinhand::utils::log_cvmat(D, "D");

  //
  std::vector<std::string> imageFilenames;
  boost::filesystem::directory_iterator itr;
  std::string fileExtension = parser.get<std::string>("extension");
  for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr) {
      if (!boost::filesystem::is_regular_file(itr->status())) {
          continue;
      }
      std::string filename = itr->path().filename().string();
      // check if file extension matches
      if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0) {
          continue;
      }

      imageFilenames.push_back(itr->path().string());
  }
  assert(imageFilenames.size() == ee_poses.rows);
  spdlog::info("loaded {} pairs img data", imageFilenames.size());
  std::sort(imageFilenames.begin(), imageFilenames.end());

  //
  std::vector<std::string> cloudFileNames;
  std::string cloudExtension = ".ply";
  for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr) {
      if (!boost::filesystem::is_regular_file(itr->status())) {
          continue;
      }
      std::string filename = itr->path().filename().string();
      // check if file extension matches
      if (filename.compare(filename.length() - cloudExtension.length(), cloudExtension.length(), cloudExtension) != 0) {
          continue;
      }

      cloudFileNames.push_back(itr->path().string());
  }
  assert(cloudFileNames.size() == ee_poses.rows);
  spdlog::info("loaded {} pairs cloud data", cloudFileNames.size());
  std::sort(cloudFileNames.begin(), cloudFileNames.end());


  //
  spdlog::info("detect corners and calc target in camera rvec + tvec ...");
  std::string board_type = parser.get<std::string>("board-type");
  cv::Size boardSize;
  boardSize.width = parser.get<int>("board-width");
  boardSize.height = parser.get<int>("board-height");
  float squareSize = parser.get<double>("square-size");
  float markerSize = parser.get<double>("marker-size");

  //
  cv::Mat objs = calib_eyeinhand::utils::genObjs(boardSize, squareSize);

  //
  std::vector<cv::Mat> Homo_target2cam;
  std::vector<cv::Mat> Homo_gripper2base;
  cv::Mat tempR, tempT, tempM;
  std::vector<cv::Mat> R_gripper2base;
  std::vector<cv::Mat> T_gripper2base;
  std::vector<cv::Mat> R_target2cam;
  std::vector<cv::Mat> T_target2cam;
  std::vector<cv::Mat> all_corners;
  std::vector<Eigen::Vector4d> all_plane_coefs;


  for(int i=0; i<imageFilenames.size(); i++){
    spdlog::info("{} start process", imageFilenames[i]);

    cv::Mat image = cv::imread(imageFilenames[i], -1);

    bool bsuc = false;
    double rerror = 0;
    cv::Mat imageCopy, corners, charucoIds, rvec, tvec;

    if("charuco" == board_type){
      bsuc = calib_eyeinhand::utils::detectCharucoCornersAndPose(boardSize, squareSize, markerSize, image, K, D, objs, imageCopy, corners, charucoIds, rvec, tvec, rerror);
    }
    else if("chess" == board_type){
      bsuc = calib_eyeinhand::utils::detectChessCornersAndPose(boardSize, image, K, D, objs, imageCopy, corners, rvec, tvec, rerror);
    }
    if (bsuc){
      cv::Mat rt_m = (cv::Mat_<double>(1, 6) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
      tempM = calib_eyeinhand::utils::attitudeVectorToMatrix(rt_m, false, "");
      Homo_target2cam.push_back(tempM);
      calib_eyeinhand::utils::HomogeneousMtr2RT(tempM, tempR, tempT);
      R_target2cam.push_back(tempR);
      T_target2cam.push_back(tempT);

      //
      tempM = calib_eyeinhand::utils::attitudeVectorToMatrix(ee_poses.row(i), false, "xyz");
      Homo_gripper2base.push_back(tempM);
      calib_eyeinhand::utils::HomogeneousMtr2RT(tempM, tempR, tempT);
      R_gripper2base.push_back(tempR);
      T_gripper2base.push_back(tempT);
      all_corners.push_back(corners);

      boost::filesystem::path filepath(imageFilenames[i]);
      spdlog::info("{} charuco corners rerror: {:.2f}", filepath.filename().string(), rerror);

      if (verbose){
        std::string out_file = outputDir + "/" + filepath.filename().string();
        cv::imwrite(out_file, imageCopy);

      // save corners as point cloud (camera coord)
        // corners in target coord
      std::vector<cv::Point3f> chessboardPoints;
      for (int i = 0; i < boardSize.height; ++i) {
          for (int j = 0; j < boardSize.width; ++j) {
              chessboardPoints.emplace_back((j+1) * squareSize/1000.0f, (i+1) * squareSize/1000.0f, 0.0f);
          }
      }
        //
      cv::Mat tmp = (cv::Mat_<double>(1, 6) << 
      tvec.at<double>(0),
      tvec.at<double>(1), 
      tvec.at<double>(2), 
      rvec.at<double>(0),
      rvec.at<double>(1), 
      rvec.at<double>(2));
      cv::Mat final_target2cam = calib_eyeinhand::utils::attitudeVectorToMatrix(tmp, false, "");

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cloud->width = chessboardPoints.size();
      cloud->height = 1;
      cloud->is_dense = false;
      cloud->points.resize(chessboardPoints.size());
      for (size_t k = 0; k < chessboardPoints.size(); ++k) {
        const auto& pt = chessboardPoints[k];
        cv::Mat pt_homo = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
        cv::Mat pt_cam_homo = final_target2cam * pt_homo;
        cloud->points[k].x = pt_cam_homo.at<double>(0, 0)*1000;
        cloud->points[k].y = pt_cam_homo.at<double>(1, 0)*1000;
        cloud->points[k].z = pt_cam_homo.at<double>(2, 0)*1000;
      }
      boost::filesystem::path filepath(imageFilenames[i]);
      std::string basename = filepath.stem().string();
      out_file = outputDir + "/corner_" + basename + ".pcd";
      pcl::io::savePCDFile(out_file, *cloud);
      }

      // //
      // const auto& cloud_file = cloudFileNames[i];
      // pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>());
      // calib_eyeinhand::utils::loadPointCloud(cloud_file, frame);
      // for (auto& point : *frame) {
      //     point.x /= 1000.0;
      //     point.y /= 1000.0;
      //     point.z /= 1000.0;
      // }

      // pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      // Eigen::Vector4d plane_coef;
      // calib_eyeinhand::utils::extractPlaneAndProjectiton(frame, plane_coef, plane_cloud);
      // all_plane_coefs.push_back(plane_coef);
      // if (verbose){
      //     boost::filesystem::path filepath(cloudFileNames[i]);
      //     std::string basename = filepath.stem().string();
      //     std::string out_file = outputDir + "/plane_" + basename + ".pcd";
      //     for (auto& point : *plane_cloud) {
      //         point.x *= 1000.0;
      //         point.y *= 1000.0;
      //         point.z *= 1000.0;
      //     }
      //     pcl::io::savePCDFile(out_file, *plane_cloud);
      // }

    }
  }

  spdlog::info("detect imgs: {} ", R_target2cam.size());
  assert(R_target2cam.size()>=3);


  // calib
  spdlog::info("start calib hand in eye by TSAI...");
  cv::calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
  Homo_cam2gripper = calib_eyeinhand::utils::RT2HomogeneousMatrix(R_cam2gripper, T_cam2gripper);
  calib_eyeinhand::utils::log_cvmat(Homo_cam2gripper, "cam2gripper(CALIB_HAND_EYE_TSAI)");
  calib_eyeinhand::utils::save_cvmat(Homo_cam2gripper, "cam2gripper(CALIB_HAND_EYE_TSAI)", outputDir);


  spdlog::info("valid chess board original point in base...");
  for(int j=0; j<objs.rows; j++){
    double error = 0;
    cv::Mat basePos;
    for (int i = 0; i < Homo_target2cam.size(); i++){
      cv::Mat chessPos = (cv::Mat_<double>(4, 1) <<objs.at<cv::Vec3f>(j)[0],objs.at<cv::Vec3f>(j)[1],0.0,1.0 );  //4*1矩阵，单独求机械臂坐标系下，标定板XYZ
      cv::Mat worldPos = Homo_gripper2base[i] * Homo_cam2gripper * Homo_target2cam[i] * chessPos;
      if(0==i)
        basePos = worldPos.clone();
      else {
        error += cv::norm(worldPos - basePos);
      }
    }
    cout << j << ": " << error/Homo_target2cam.size()*1000 << " mm" << endl;
    calib_eyeinhand::utils::save_error(j, error/Homo_target2cam.size()*1000, outputDir);
  }

  // //
  cv::Mat rvec, tvec;
 
  // opt by ceres
  spdlog::info("start opt by ceres ...");
  double* parameters_ = new double[6*(R_target2cam.size())];
  for(int i=0; i<R_target2cam.size(); i++){
    cv::Rodrigues(R_target2cam[i], rvec);
    parameters_[i*6 + 0] = rvec.at<double>(0);
    parameters_[i*6 + 1] = rvec.at<double>(1);
    parameters_[i*6 + 2] = rvec.at<double>(2);
    parameters_[i*6 + 3] = T_target2cam[i].at<double>(0);
    parameters_[i*6 + 4] = T_target2cam[i].at<double>(1);
    parameters_[i*6 + 5] = T_target2cam[i].at<double>(2);
  }
  calib_eyeinhand::utils::HomogeneousMtr2RT(Homo_cam2gripper, tempR, tempT);
  cv::Rodrigues(tempR, rvec);
  double parameters_cam2gripper[6];
  parameters_cam2gripper[0] = rvec.at<double>(0);
  parameters_cam2gripper[1] = rvec.at<double>(1);
  parameters_cam2gripper[2] = rvec.at<double>(2);
  parameters_cam2gripper[3] = tempT.at<double>(0);
  parameters_cam2gripper[4] = tempT.at<double>(1);
  parameters_cam2gripper[5] = tempT.at<double>(2);

  double parameters_KFxy[2];
  parameters_KFxy[0] = K.at<float>(0,0);
  parameters_KFxy[1] = K.at<float>(1,1);

  ceres::Problem problem;
  //
  for(int i=0; i<R_target2cam.size(); i++){
    for(int j=0; j<R_target2cam.size(); j++){
      for(int k=0; k<objs.rows; k++){
 
        Eigen::Vector3d obj(objs.at<cv::Vec3f>(k)[0], objs.at<cv::Vec3f>(k)[1], objs.at<cv::Vec3f>(k)[2]);
        Eigen::Vector2d corner(all_corners[j].at<cv::Vec2f>(k)[0], all_corners[j].at<cv::Vec2f>(k)[1]);
        Eigen::Matrix3d K_Eigen;
        cv::cv2eigen(K, K_Eigen);
        Eigen::Matrix3d gr1, gr2;
        Eigen::Vector3d gt1, gt2;
        cv::cv2eigen(R_gripper2base[i], gr1);
        cv::cv2eigen(R_gripper2base[j], gr2);
        cv::cv2eigen(T_gripper2base[i], gt1);
        cv::cv2eigen(T_gripper2base[j], gt2);

        ceres::CostFunction* cost_funciton = HandinEyeReprojectionError::Create(obj, corner, K_Eigen, gr1, gt1, gr2, gt2);
        ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
        if(i==j)
          for(int cnt=0; cnt<R_target2cam.size(); cnt++)
            problem.AddResidualBlock(cost_funciton, lossFunction, parameters_+i*6, parameters_cam2gripper, parameters_KFxy);
        else
            problem.AddResidualBlock(cost_funciton, lossFunction, parameters_+i*6, parameters_cam2gripper, parameters_KFxy);

      }
    }
  }
  // problem.SetParameterBlockConstant(parameters_);
  problem.SetParameterBlockConstant(parameters_KFxy);
  
  // for(int i=0;i<all_plane_coefs.size();i++){
  //   for(int k=0; k<objs.rows; k++){
  //       Eigen::Vector3d obj(objs.at<cv::Vec3f>(k)[0], objs.at<cv::Vec3f>(k)[1], objs.at<cv::Vec3f>(k)[2]);
  //       ceres::CostFunction* cost_funciton = Point2PlaneError::Create(all_plane_coefs[i], obj, false);
  //       ceres::LossFunction *lossFunction = new ceres::CauchyLoss(1.0);
  //       for(int j=0; j<all_plane_coefs.size()*all_plane_coefs.size();j++)
  //         problem.AddResidualBlock(cost_funciton, lossFunction, parameters_+i*6);
  //   }
  // }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // // corners in target coord
  // std::vector<cv::Point3f> chessboardPoints;
  // for (int i = 0; i < boardSize.height; ++i) {
  //     for (int j = 0; j < boardSize.width; ++j) {
  //         chessboardPoints.emplace_back((j+1) * squareSize/1000.0f, (i+1) * squareSize/1000.0f, 0.0f);
  //     }
  // }

  K.at<float>(0,0) = parameters_KFxy[0];
  K.at<float>(1,1) = parameters_KFxy[1];

  spdlog::info("KFxy: {}, {}", parameters_KFxy[0] ,parameters_KFxy[1]);
  calib_eyeinhand::utils::log_cvmat(K, "K(CERES)");

  cv::Mat final_cam2gripper = (cv::Mat_<double>(1, 6) << 
    parameters_cam2gripper[3], 
    parameters_cam2gripper[4], 
    parameters_cam2gripper[5], 
    parameters_cam2gripper[0], 
    parameters_cam2gripper[1], 
    parameters_cam2gripper[2]);
  Homo_cam2gripper = calib_eyeinhand::utils::attitudeVectorToMatrix(final_cam2gripper, false, "");
  calib_eyeinhand::utils::log_cvmat(Homo_cam2gripper, "cam2gripper(CERES)");
  calib_eyeinhand::utils::save_cvmat(Homo_cam2gripper, "cam2gripper(CERES)", outputDir);

  spdlog::info("valid chess board original point in base [after opt]...");
  for(int j=0; j<objs.rows; j++) {
    double error = 0;
    cv::Mat basePos;
    for (int i = 0; i < Homo_target2cam.size(); i++){
      cv::Mat tmp = (cv::Mat_<double>(1, 6) << 
        parameters_[i*6+3], 
        parameters_[i*6+4], 
        parameters_[i*6+5], 
        parameters_[i*6+0], 
        parameters_[i*6+1], 
        parameters_[i*6+2]);
      cv::Mat final_target2cam = calib_eyeinhand::utils::attitudeVectorToMatrix(tmp, false, "");
      cv::Mat chessPos = (cv::Mat_<double>(4, 1) <<objs.at<cv::Vec3f>(j)[0],objs.at<cv::Vec3f>(j)[1],0.0,1.0 );
      cv::Mat worldPos = Homo_gripper2base[i] * Homo_cam2gripper * final_target2cam * chessPos;
      if(0==i)
        basePos = worldPos.clone();
      else {
        error += cv::norm(worldPos - basePos);
      }
    }
    cout << j << ": " << error/Homo_target2cam.size()*1000 << " mm" << endl;
    calib_eyeinhand::utils::save_error(j, error/Homo_target2cam.size()*1000, outputDir);
  }


  delete[] parameters_;


  // output

  return 0;
}