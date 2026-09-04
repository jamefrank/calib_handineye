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

#include <filesystem>
#include <fstream>
#include <iostream>
using namespace std;


struct HandEyeClosureMetrics
{
  double translation_rmse_m = 0.0;
  double rotation_rmse_deg = 0.0;
  std::vector<double> translation_errors_m;
  std::vector<double> rotation_errors_deg;
};

struct MethodComparison
{
  std::string name;
  cv::Mat cam2gripper;
  double translation_rmse_m = 0.0;
  double rotation_rmse_deg = 0.0;
  std::string note;
};


cv::Mat averageBoardPoseInBase(const std::vector<cv::Mat>& board_poses)
{
  CV_Assert(!board_poses.empty());

  cv::Mat avg = board_poses[0].clone();
  for (int iter = 0; iter < 50; ++iter)
  {
    cv::Mat avg_inv = avg.inv();
    cv::Mat sum_t = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat sum_r = cv::Mat::zeros(3, 1, CV_64FC1);

    for (const auto& pose : board_poses)
    {
      cv::Mat delta = avg_inv * pose;
      cv::Mat t = delta(cv::Rect(3, 0, 1, 3)).clone();
      cv::Mat R = delta(cv::Rect(0, 0, 3, 3)).clone();
      cv::Mat rvec;
      cv::Rodrigues(R, rvec);
      sum_t += t;
      sum_r += rvec;
    }

    cv::Mat mean_t = sum_t / static_cast<double>(board_poses.size());
    cv::Mat mean_r = sum_r / static_cast<double>(board_poses.size());

    if (cv::norm(mean_t) < 1e-9 && cv::norm(mean_r) < 1e-9)
      break;

    cv::Mat R;
    cv::Rodrigues(mean_r, R);
    cv::Mat update = cv::Mat::eye(4, 4, CV_64FC1);
    R.copyTo(update(cv::Rect(0, 0, 3, 3)));
    mean_t.copyTo(update(cv::Rect(3, 0, 1, 3)));
    avg = avg * update;
  }

  return avg;
}


HandEyeClosureMetrics computeHandEyeClosureMetrics(
  const std::vector<cv::Mat>& Homo_gripper2base,
  const cv::Mat& Homo_cam2gripper,
  const std::vector<cv::Mat>& Homo_target2cam)
{
  CV_Assert(Homo_gripper2base.size() == Homo_target2cam.size());

  std::vector<cv::Mat> board_poses;
  board_poses.reserve(Homo_target2cam.size());
  for (size_t i = 0; i < Homo_target2cam.size(); ++i)
  {
    board_poses.push_back(
      Homo_gripper2base[i] * Homo_cam2gripper * Homo_target2cam[i]);
  }

  cv::Mat T_base_board = averageBoardPoseInBase(board_poses);
  cv::Mat T_base_board_inv = T_base_board.inv();

  HandEyeClosureMetrics metrics;
  double sum_translation_sq = 0.0;
  double sum_rotation_sq = 0.0;

  for (const auto& pose : board_poses)
  {
    cv::Mat delta = T_base_board_inv * pose;
    cv::Mat t = delta(cv::Rect(3, 0, 1, 3));
    cv::Mat R = delta(cv::Rect(0, 0, 3, 3));
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    double translation_error = cv::norm(t);
    double rotation_error_deg = cv::norm(rvec) * 180.0 / CV_PI;

    metrics.translation_errors_m.push_back(translation_error);
    metrics.rotation_errors_deg.push_back(rotation_error_deg);
    sum_translation_sq += translation_error * translation_error;
    sum_rotation_sq += rotation_error_deg * rotation_error_deg;
  }

  const double n = static_cast<double>(board_poses.size());
  metrics.translation_rmse_m = std::sqrt(sum_translation_sq / n);
  metrics.rotation_rmse_deg = std::sqrt(sum_rotation_sq / n);
  return metrics;
}


HandEyeClosureMetrics computeHandEyeClosureMetricsWithReference(
  const std::vector<cv::Mat>& Homo_gripper2base,
  const cv::Mat& Homo_cam2gripper,
  const std::vector<cv::Mat>& Homo_target2cam,
  const cv::Mat& T_base_board)
{
  CV_Assert(Homo_gripper2base.size() == Homo_target2cam.size());

  cv::Mat T_base_board_inv = T_base_board.inv();
  HandEyeClosureMetrics metrics;
  double sum_translation_sq = 0.0;
  double sum_rotation_sq = 0.0;

  for (size_t i = 0; i < Homo_target2cam.size(); ++i)
  {
    cv::Mat pose =
        Homo_gripper2base[i] * Homo_cam2gripper * Homo_target2cam[i];
    cv::Mat delta = T_base_board_inv * pose;
    cv::Mat t = delta(cv::Rect(3, 0, 1, 3));
    cv::Mat R = delta(cv::Rect(0, 0, 3, 3));
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    double translation_error = cv::norm(t);
    double rotation_error_deg = cv::norm(rvec) * 180.0 / CV_PI;

    metrics.translation_errors_m.push_back(translation_error);
    metrics.rotation_errors_deg.push_back(rotation_error_deg);
    sum_translation_sq += translation_error * translation_error;
    sum_rotation_sq += rotation_error_deg * rotation_error_deg;
  }

  const double n = static_cast<double>(Homo_target2cam.size());
  metrics.translation_rmse_m = std::sqrt(sum_translation_sq / n);
  metrics.rotation_rmse_deg = std::sqrt(sum_rotation_sq / n);
  return metrics;
}


void saveHandEyeClosureMetrics(
  const std::string& label,
  const HandEyeClosureMetrics& metrics,
  const std::string& outputDir)
{
  std::ofstream file(outputDir + "/calibration.txt", std::ios::app);
  if (!file.is_open())
  {
    std::cerr << "无法打开文件 " << outputDir + "/calibration.txt" << std::endl;
    return;
  }

  file << std::endl;
  file << label << " hand_eye_closure_translation_rmse_m = "
       << metrics.translation_rmse_m << std::endl;
  file << label << " hand_eye_closure_rotation_rmse_deg = "
       << metrics.rotation_rmse_deg << std::endl;

  for (size_t i = 0; i < metrics.translation_errors_m.size(); ++i)
  {
    file << label << " sample " << (i + 1)
         << ": translation_error_m = " << metrics.translation_errors_m[i]
         << ", rotation_error_deg = " << metrics.rotation_errors_deg[i]
         << std::endl;
  }

  file.close();
}


void computeAndSaveOpenCVHandEyeMethod(
  cv::HandEyeCalibrationMethod method,
  const std::string& method_name,
  const std::vector<cv::Mat>& R_gripper2base,
  const std::vector<cv::Mat>& T_gripper2base,
  const std::vector<cv::Mat>& R_target2cam,
  const std::vector<cv::Mat>& T_target2cam,
  const std::vector<cv::Mat>& Homo_gripper2base,
  const std::vector<cv::Mat>& Homo_target2cam,
  const std::string& outputDir)
{
  cv::Mat R_cam2gripper;
  cv::Mat T_cam2gripper;
  cv::calibrateHandEye(
      R_gripper2base, T_gripper2base,
      R_target2cam, T_target2cam,
      R_cam2gripper, T_cam2gripper,
      method);

  cv::Mat Homo_cam2gripper =
      calib_eyeinhand::utils::RT2HomogeneousMatrix(
          R_cam2gripper, T_cam2gripper);

  const std::string matrix_name = "cam2gripper(OPENCV_" + method_name + ")";
  calib_eyeinhand::utils::log_cvmat(Homo_cam2gripper, matrix_name);
  calib_eyeinhand::utils::save_cvmat(
      Homo_cam2gripper, matrix_name, outputDir);

  HandEyeClosureMetrics metrics = computeHandEyeClosureMetrics(
      Homo_gripper2base, Homo_cam2gripper, Homo_target2cam);
  spdlog::info(
      "{} hand-eye closure translation_rmse_m: {}, rotation_rmse_deg: {}",
      method_name, metrics.translation_rmse_m, metrics.rotation_rmse_deg);
  saveHandEyeClosureMetrics(method_name, metrics, outputDir);
}


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
  parser.add("intrinsics-fx-only", '\0', "BA: only optimize fx/fy (fix cx/cy and distortion)");
  parser.add("full-ba", '\0', "BA: jointly optimize per-view board poses too");
  parser.add("fix-intrinsics", '\0', "full BA: keep intrinsics completely fixed");
  parser.add("opt-arm-pose", '\0', "BA: refine robot end-effector poses with a prior");
  parser.add<double>("arm-pose-weight", '\0', "prior weight for end-effector pose refinement", false, 1000.0);
  parser.add("pairwise-opt-intrinsics", '\0', "pairwise BA: also optimize intrinsics");

  parser.parse_check(argc, argv);
  bool verbose = parser.exist("verbose");
  bool intrinsics_fx_only = parser.exist("intrinsics-fx-only");
  bool full_ba = parser.exist("full-ba");
  bool fix_intrinsics = parser.exist("fix-intrinsics");
  bool opt_arm_pose = parser.exist("opt-arm-pose");
  double arm_pose_weight = parser.get<double>("arm-pose-weight");
  bool pairwise_opt_intrinsics = parser.exist("pairwise-opt-intrinsics");

  //
  cv::Mat R_cam2gripper = cv::Mat(3,3,CV_64FC1);				//相机与机械臂末端坐标系的旋转矩阵与平移矩阵
  cv::Mat T_cam2gripper = cv::Mat(3,1,CV_64FC1);
  cv::Mat Homo_cam2gripper = cv::Mat(4,4,CV_64FC1);

  //
  spdlog::info("loading data...");
  std::string inputDir = parser.get<std::string>("input");
  std::string outputDir = parser.get<std::string>("output");
  // 输出目录不存在时自动创建，避免后续写 calibration.txt 失败
  std::filesystem::create_directories(outputDir);
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
  cv::Mat Homo_cam2gripper_tsai = Homo_cam2gripper.clone();

  HandEyeClosureMetrics tsa_metrics = computeHandEyeClosureMetrics(
      Homo_gripper2base, Homo_cam2gripper, Homo_target2cam);
  spdlog::info(
      "TSAI hand-eye closure translation_rmse_m: {}, rotation_rmse_deg: {}",
      tsa_metrics.translation_rmse_m, tsa_metrics.rotation_rmse_deg);
  saveHandEyeClosureMetrics("TSAI", tsa_metrics, outputDir);


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

  computeAndSaveOpenCVHandEyeMethod(
      cv::CALIB_HAND_EYE_PARK, "PARK",
      R_gripper2base, T_gripper2base,
      R_target2cam, T_target2cam,
      Homo_gripper2base, Homo_target2cam, outputDir);

  computeAndSaveOpenCVHandEyeMethod(
      cv::CALIB_HAND_EYE_HORAUD, "HORAUD",
      R_gripper2base, T_gripper2base,
      R_target2cam, T_target2cam,
      Homo_gripper2base, Homo_target2cam, outputDir);

  computeAndSaveOpenCVHandEyeMethod(
      cv::CALIB_HAND_EYE_ANDREFF, "ANDREFF",
      R_gripper2base, T_gripper2base,
      R_target2cam, T_target2cam,
      Homo_gripper2base, Homo_target2cam, outputDir);

  computeAndSaveOpenCVHandEyeMethod(
      cv::CALIB_HAND_EYE_DANIILIDIS, "DANIILIDIS",
      R_gripper2base, T_gripper2base,
      R_target2cam, T_target2cam,
      Homo_gripper2base, Homo_target2cam, outputDir);

  spdlog::info("start calib hand in eye by TASHAN_ROBOT_METHOD...");
  std::vector<Eigen::Matrix3d> R_base_ee_list;
  std::vector<Eigen::Vector3d> t_base_ee_list;
  std::vector<Eigen::Matrix3d> R_camera_board_list;
  std::vector<Eigen::Vector3d> t_camera_board_list;
  R_base_ee_list.reserve(R_gripper2base.size());
  t_base_ee_list.reserve(R_gripper2base.size());
  R_camera_board_list.reserve(R_target2cam.size());
  t_camera_board_list.reserve(R_target2cam.size());
  for (int i = 0; i < R_gripper2base.size(); i++)
  {
    cv::Mat R_be, t_be, R_cb, t_cb;
    calib_eyeinhand::utils::HomogeneousMtr2RT(Homo_gripper2base[i], R_be, t_be);
    calib_eyeinhand::utils::HomogeneousMtr2RT(Homo_target2cam[i], R_cb, t_cb);

    Eigen::Matrix3d R_be_e, R_cb_e;
    Eigen::Vector3d t_be_e, t_cb_e;
    cv::cv2eigen(R_be, R_be_e);
    cv::cv2eigen(t_be, t_be_e);
    cv::cv2eigen(R_cb, R_cb_e);
    cv::cv2eigen(t_cb, t_cb_e);

    R_base_ee_list.push_back(R_be_e);
    t_base_ee_list.push_back(t_be_e);
    R_camera_board_list.push_back(R_cb_e);
    t_camera_board_list.push_back(t_cb_e);
  }

  double tashan_ee_camera[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  cv::Mat init_board = Homo_gripper2base[0] * Homo_target2cam[0];
  cv::Mat init_R, init_t;
  calib_eyeinhand::utils::HomogeneousMtr2RT(init_board, init_R, init_t);
  cv::Mat init_rvec;
  cv::Rodrigues(init_R, init_rvec);
  double tashan_base_board[6] = {
      init_rvec.at<double>(0),
      init_rvec.at<double>(1),
      init_rvec.at<double>(2),
      init_t.at<double>(0),
      init_t.at<double>(1),
      init_t.at<double>(2)};

  ceres::Problem tashan_problem;
  for (int i = 0; i < R_base_ee_list.size(); i++)
  {
    ceres::CostFunction* cost_function =
        HandEyeBoardOnBaseClosureError::Create(
            R_base_ee_list[i], t_base_ee_list[i],
            R_camera_board_list[i], t_camera_board_list[i]);
    tashan_problem.AddResidualBlock(
        cost_function, nullptr, tashan_ee_camera, tashan_base_board);
  }

  ceres::Solver::Options tashan_options;
  tashan_options.linear_solver_type = ceres::DENSE_SCHUR;
  tashan_options.minimizer_progress_to_stdout = false;
  tashan_options.max_num_iterations = 200;
  tashan_options.num_threads = 8;
  ceres::Solver::Summary tashan_summary;
  ceres::Solve(tashan_options, &tashan_problem, &tashan_summary);
  std::cout << tashan_summary.FullReport() << "\n";

  cv::Mat tashan_ee_rvec = (cv::Mat_<double>(3, 1) <<
      tashan_ee_camera[0], tashan_ee_camera[1], tashan_ee_camera[2]);
  cv::Mat tashan_ee_R;
  cv::Rodrigues(tashan_ee_rvec, tashan_ee_R);
  cv::Mat tashan_ee_t = (cv::Mat_<double>(3, 1) <<
      tashan_ee_camera[3], tashan_ee_camera[4], tashan_ee_camera[5]);
  cv::Mat Homo_cam2gripper_tashan =
      calib_eyeinhand::utils::RT2HomogeneousMatrix(tashan_ee_R, tashan_ee_t);
  calib_eyeinhand::utils::log_cvmat(
      Homo_cam2gripper_tashan, "cam2gripper(TASHAN_ROBOT_METHOD)");
  calib_eyeinhand::utils::save_cvmat(
      Homo_cam2gripper_tashan, "cam2gripper(TASHAN_ROBOT_METHOD)", outputDir);

  cv::Mat tashan_board_rvec = (cv::Mat_<double>(3, 1) <<
      tashan_base_board[0], tashan_base_board[1], tashan_base_board[2]);
  cv::Mat tashan_board_R;
  cv::Rodrigues(tashan_board_rvec, tashan_board_R);
  cv::Mat tashan_board_t = (cv::Mat_<double>(3, 1) <<
      tashan_base_board[3], tashan_base_board[4], tashan_base_board[5]);
  cv::Mat Homo_board2base_tashan =
      calib_eyeinhand::utils::RT2HomogeneousMatrix(tashan_board_R, tashan_board_t);
  calib_eyeinhand::utils::log_cvmat(
      Homo_board2base_tashan, "board2base(TASHAN_ROBOT_METHOD)");
  calib_eyeinhand::utils::save_cvmat(
      Homo_board2base_tashan, "board2base(TASHAN_ROBOT_METHOD)", outputDir);

  HandEyeClosureMetrics tashan_metrics =
      computeHandEyeClosureMetrics(
          Homo_gripper2base, Homo_cam2gripper_tashan, Homo_target2cam);
  spdlog::info(
      "TASHAN hand-eye closure translation_rmse_m: {}, rotation_rmse_deg: {}",
      tashan_metrics.translation_rmse_m, tashan_metrics.rotation_rmse_deg);
  saveHandEyeClosureMetrics("TASHAN", tashan_metrics, outputDir);

  // //
  cv::Mat rvec, tvec;

  // 原始 i-j 两两重投影 CERES：每张图 target2cam 自由 + cam2gripper，对角项重复加权重
  spdlog::info("start original pairwise (i-j) CERES ...");
  double* pairwise_parameters = new double[6 * R_target2cam.size()];
  for (int i = 0; i < R_target2cam.size(); i++)
  {
    cv::Rodrigues(R_target2cam[i], rvec);
    pairwise_parameters[i*6 + 0] = rvec.at<double>(0);
    pairwise_parameters[i*6 + 1] = rvec.at<double>(1);
    pairwise_parameters[i*6 + 2] = rvec.at<double>(2);
    pairwise_parameters[i*6 + 3] = T_target2cam[i].at<double>(0);
    pairwise_parameters[i*6 + 4] = T_target2cam[i].at<double>(1);
    pairwise_parameters[i*6 + 5] = T_target2cam[i].at<double>(2);
  }
  calib_eyeinhand::utils::HomogeneousMtr2RT(Homo_cam2gripper, tempR, tempT);
  cv::Rodrigues(tempR, rvec);
  double pairwise_cam2gripper[6];
  pairwise_cam2gripper[0] = rvec.at<double>(0);
  pairwise_cam2gripper[1] = rvec.at<double>(1);
  pairwise_cam2gripper[2] = rvec.at<double>(2);
  pairwise_cam2gripper[3] = tempT.at<double>(0);
  pairwise_cam2gripper[4] = tempT.at<double>(1);
  pairwise_cam2gripper[5] = tempT.at<double>(2);
  double pairwise_KFxy[2];
  pairwise_KFxy[0] = K.at<float>(0,0);
  pairwise_KFxy[1] = K.at<float>(1,1);
  double pairwise_intrinsics[9];
  pairwise_intrinsics[0] = K.at<float>(0,0);
  pairwise_intrinsics[1] = K.at<float>(1,1);
  pairwise_intrinsics[2] = K.at<float>(0,2);
  pairwise_intrinsics[3] = K.at<float>(1,2);
  pairwise_intrinsics[4] = D.at<float>(0);
  pairwise_intrinsics[5] = D.at<float>(1);
  pairwise_intrinsics[6] = D.at<float>(2);
  pairwise_intrinsics[7] = D.at<float>(3);
  pairwise_intrinsics[8] = D.at<float>(4);
  double pairwise_base_board[6];
  for (int a = 0; a < 6; ++a)
    pairwise_base_board[a] = tashan_base_board[a];

  ceres::Problem pairwise_problem;
  for (int i = 0; i < R_target2cam.size(); i++)
  {
    for (int j = 0; j < R_target2cam.size(); j++)
    {
      for (int k = 0; k < objs.rows; k++)
      {
        Eigen::Vector3d obj(
            objs.at<cv::Vec3f>(k)[0],
            objs.at<cv::Vec3f>(k)[1],
            objs.at<cv::Vec3f>(k)[2]);
        Eigen::Vector2d corner(
            all_corners[j].at<cv::Vec2f>(k)[0],
            all_corners[j].at<cv::Vec2f>(k)[1]);
        Eigen::Matrix3d K_Eigen;
        cv::cv2eigen(K, K_Eigen);
        Eigen::Matrix3d gr1, gr2;
        Eigen::Vector3d gt1, gt2;
        cv::cv2eigen(R_gripper2base[i], gr1);
        cv::cv2eigen(R_gripper2base[j], gr2);
        cv::cv2eigen(T_gripper2base[i], gt1);
        cv::cv2eigen(T_gripper2base[j], gt2);

        ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
        if (pairwise_opt_intrinsics)
        {
          ceres::CostFunction* cost_function =
              HandinEyeReprojectionErrorIntrinsics::Create(
                  obj, corner, gr1, gt1, gr2, gt2);
          if (i == j)
          {
            for (int cnt = 0; cnt < R_target2cam.size(); cnt++)
              pairwise_problem.AddResidualBlock(
                  cost_function, loss_function,
                  pairwise_parameters + i*6,
                  pairwise_cam2gripper, pairwise_intrinsics);
          }
          else
          {
            pairwise_problem.AddResidualBlock(
                cost_function, loss_function,
                pairwise_parameters + i*6,
                pairwise_cam2gripper, pairwise_intrinsics);
          }
        }
        else
        {
          ceres::CostFunction* cost_function =
              HandinEyeReprojectionError::Create(
                  obj, corner, K_Eigen, gr1, gt1, gr2, gt2);
          if (i == j)
          {
            for (int cnt = 0; cnt < R_target2cam.size(); cnt++)
              pairwise_problem.AddResidualBlock(
                  cost_function, loss_function,
                  pairwise_parameters + i*6,
                  pairwise_cam2gripper, pairwise_KFxy);
          }
          else
          {
            pairwise_problem.AddResidualBlock(
                cost_function, loss_function,
                pairwise_parameters + i*6,
                pairwise_cam2gripper, pairwise_KFxy);
          }
        }
      }
    }
  }
  // 融合 full-BA 思想：给 pairwise 加 base_board 和手眼闭环约束
  const double pairwise_closure_weight = 1000.0;
  for (int i = 0; i < R_target2cam.size(); ++i)
  {
    Eigen::Matrix3d R_be;
    Eigen::Vector3d t_be;
    cv::cv2eigen(R_gripper2base[i], R_be);
    cv::cv2eigen(T_gripper2base[i], t_be);
    ceres::CostFunction* closure =
        HandEyeBoardPoseClosureError::Create(
            R_be, t_be, pairwise_closure_weight);
    pairwise_problem.AddResidualBlock(
        closure, nullptr,
        pairwise_cam2gripper, pairwise_base_board,
        pairwise_parameters + i*6);
  }
  if (!pairwise_opt_intrinsics)
    pairwise_problem.SetParameterBlockConstant(pairwise_KFxy);

  ceres::Solver::Options poptions;
  poptions.linear_solver_type = ceres::DENSE_SCHUR;
  poptions.minimizer_progress_to_stdout = false;
  poptions.max_num_iterations = 200;
  poptions.num_threads = 8;
  ceres::Solver::Summary psummary;
  ceres::Solve(poptions, &pairwise_problem, &psummary);
  std::cout << psummary.FullReport() << "\n";
  spdlog::info("PAIRWISE-CERES final cost: {}", psummary.final_cost);

  cv::Mat pairwise_c2g = (cv::Mat_<double>(1, 6) <<
      pairwise_cam2gripper[3],
      pairwise_cam2gripper[4],
      pairwise_cam2gripper[5],
      pairwise_cam2gripper[0],
      pairwise_cam2gripper[1],
      pairwise_cam2gripper[2]);
  Homo_cam2gripper =
      calib_eyeinhand::utils::attitudeVectorToMatrix(pairwise_c2g, false, "");
  calib_eyeinhand::utils::log_cvmat(
      Homo_cam2gripper, "cam2gripper(CERES_PAIRWISE)");
  calib_eyeinhand::utils::save_cvmat(
      Homo_cam2gripper, "cam2gripper(CERES_PAIRWISE)", outputDir);

  std::vector<cv::Mat> Homo_target2cam_pairwise;
  for (int i = 0; i < R_target2cam.size(); i++)
  {
    cv::Mat tmp = (cv::Mat_<double>(1, 6) <<
        pairwise_parameters[i*6 + 3],
        pairwise_parameters[i*6 + 4],
        pairwise_parameters[i*6 + 5],
        pairwise_parameters[i*6 + 0],
        pairwise_parameters[i*6 + 1],
        pairwise_parameters[i*6 + 2]);
    Homo_target2cam_pairwise.push_back(
        calib_eyeinhand::utils::attitudeVectorToMatrix(tmp, false, ""));
  }
  HandEyeClosureMetrics pairwise_metrics = computeHandEyeClosureMetrics(
      Homo_gripper2base, Homo_cam2gripper, Homo_target2cam_pairwise);
  spdlog::info(
      "PAIRWISE-CERES closure translation_rmse_m: {}, rotation_rmse_deg: {}",
      pairwise_metrics.translation_rmse_m, pairwise_metrics.rotation_rmse_deg);
  saveHandEyeClosureMetrics("PAIRWISE-CERES", pairwise_metrics, outputDir);
  delete[] pairwise_parameters;

  // COLMAP-style bundle adjustment：联合优化 T_ee_camera / T_base_board / 内参(含畸变)
  spdlog::info("start opt by ceres (COLMAP-style BA with intrinsics) ...");

  // 每张图 target2cam 初值（供后续棋盘点验证使用）
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

  // 初值：X、T_base_board 用 TASHAN 解，内参用当前配置
  double ba_ee_camera[6];
  double ba_base_board[6];
  for (int a = 0; a < 6; ++a)
  {
    ba_ee_camera[a]   = tashan_ee_camera[a];
    ba_base_board[a]  = tashan_base_board[a];
  }
  double ba_intrinsics[9];
  ba_intrinsics[0] = K.at<float>(0,0);
  ba_intrinsics[1] = K.at<float>(1,1);
  ba_intrinsics[2] = K.at<float>(0,2);
  ba_intrinsics[3] = K.at<float>(1,2);
  ba_intrinsics[4] = D.at<float>(0);
  ba_intrinsics[5] = D.at<float>(1);
  ba_intrinsics[6] = D.at<float>(2);
  ba_intrinsics[7] = D.at<float>(3);
  ba_intrinsics[8] = D.at<float>(4);
  double ba_fxy[2];
  ba_fxy[0] = ba_intrinsics[0];
  ba_fxy[1] = ba_intrinsics[1];
  Eigen::Matrix3d K_eigen;
  cv::cv2eigen(K, K_eigen);
  Eigen::VectorXd D_eigen(D.cols);
  for (int a = 0; a < D.cols; ++a)
    D_eigen(a) = D.at<float>(a);

  ceres::Problem ba_problem;
  for (int i = 0; i < R_target2cam.size(); ++i)
  {
    Eigen::Matrix3d R_be;
    Eigen::Vector3d t_be;
    cv::cv2eigen(R_gripper2base[i], R_be);
    cv::cv2eigen(T_gripper2base[i], t_be);
    for (int k = 0; k < objs.rows; ++k)
    {
      Eigen::Vector3d obj(
          objs.at<cv::Vec3f>(k)[0],
          objs.at<cv::Vec3f>(k)[1],
          objs.at<cv::Vec3f>(k)[2]);
      Eigen::Vector2d corner(
          all_corners[i].at<cv::Vec2f>(k)[0],
          all_corners[i].at<cv::Vec2f>(k)[1]);
      ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
      if (intrinsics_fx_only)
      {
        ceres::CostFunction* cost_function =
            HandEyeBoardOnBaseReprojectionErrorFx::Create(
                obj, corner, R_be, t_be, K_eigen, D_eigen);
        ba_problem.AddResidualBlock(
            cost_function, loss_function,
            ba_ee_camera, ba_base_board, ba_fxy);
      }
      else
      {
        ceres::CostFunction* cost_function =
            HandEyeBoardOnBaseReprojectionError::Create(
                obj, corner, R_be, t_be);
        ba_problem.AddResidualBlock(
            cost_function, loss_function,
            ba_ee_camera, ba_base_board, ba_intrinsics);
      }
    }
  }

  // 联合优化：加入 3D 手眼闭环残差（加权，使其与像素残差量级可比）
  const double closure_weight = 1000.0;
  for (int i = 0; i < R_base_ee_list.size(); ++i)
  {
    ceres::CostFunction* closure_cost =
        HandEyeBoardOnBaseClosureError::Create(
            R_base_ee_list[i], t_base_ee_list[i],
            R_camera_board_list[i], t_camera_board_list[i],
            closure_weight);
    ba_problem.AddResidualBlock(
        closure_cost, nullptr, ba_ee_camera, ba_base_board);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &ba_problem, &summary);
  std::cout << summary.FullReport() << "\n";
  spdlog::info("COLMAP-BA final cost: {}", summary.final_cost);
  if (intrinsics_fx_only)
  {
    ba_intrinsics[0] = ba_fxy[0];
    ba_intrinsics[1] = ba_fxy[1];
  }

  spdlog::info(
      "COLMAP-BA intrinsics: fx={}, fy={}, cx={}, cy={}",
      ba_intrinsics[0], ba_intrinsics[1], ba_intrinsics[2], ba_intrinsics[3]);
  spdlog::info(
      "COLMAP-BA distortion: k1={}, k2={}, p1={}, p2={}, k3={}",
      ba_intrinsics[4], ba_intrinsics[5], ba_intrinsics[6],
      ba_intrinsics[7], ba_intrinsics[8]);

  if (full_ba)
  {
    spdlog::info("start full BA (per-view board poses free) ...");
    const int nviews = R_target2cam.size();
    double* full_board_poses = new double[6 * nviews];
    for (int i = 0; i < nviews; ++i)
      for (int a = 0; a < 6; ++a)
        full_board_poses[i * 6 + a] = parameters_[i * 6 + a];

    double full_ee[6];
    double full_bb[6];
    double full_intr[9];
    for (int a = 0; a < 6; ++a)
    {
      full_ee[a] = ba_ee_camera[a];
      full_bb[a] = ba_base_board[a];
    }
    for (int a = 0; a < 9; ++a)
      full_intr[a] = ba_intrinsics[a];
    if (fix_intrinsics)
    {
      full_intr[0] = K.at<float>(0,0);
      full_intr[1] = K.at<float>(1,1);
      full_intr[2] = K.at<float>(0,2);
      full_intr[3] = K.at<float>(1,2);
      full_intr[4] = D.at<float>(0);
      full_intr[5] = D.at<float>(1);
      full_intr[6] = D.at<float>(2);
      full_intr[7] = D.at<float>(3);
      full_intr[8] = D.at<float>(4);
    }
    double full_fxy[2];
    full_fxy[0] = full_intr[0];
    full_fxy[1] = full_intr[1];

    ceres::Problem full_problem;
    for (int i = 0; i < nviews; ++i)
    {
      for (int k = 0; k < objs.rows; ++k)
      {
        Eigen::Vector3d obj(
            objs.at<cv::Vec3f>(k)[0],
            objs.at<cv::Vec3f>(k)[1],
            objs.at<cv::Vec3f>(k)[2]);
        Eigen::Vector2d corner(
            all_corners[i].at<cv::Vec2f>(k)[0],
            all_corners[i].at<cv::Vec2f>(k)[1]);
        ceres::LossFunction* loss = new ceres::CauchyLoss(1.0);
        if (fix_intrinsics)
        {
          ceres::CostFunction* reproj =
              HandEyeBoardPoseReprojectionErrorFixed::Create(
                  obj, corner, K_eigen, D_eigen);
          full_problem.AddResidualBlock(
              reproj, loss, full_board_poses + i * 6);
        }
        else if (intrinsics_fx_only)
        {
          ceres::CostFunction* reproj =
              HandEyeBoardPoseReprojectionErrorFx::Create(
                  obj, corner, K_eigen, D_eigen);
          full_problem.AddResidualBlock(
              reproj, loss, full_board_poses + i * 6, full_fxy);
        }
        else
        {
          ceres::CostFunction* reproj =
              HandEyeBoardPoseReprojectionError::Create(obj, corner);
          full_problem.AddResidualBlock(
              reproj, loss, full_board_poses + i * 6, full_intr);
        }
      }
    }
    for (int i = 0; i < nviews; ++i)
    {
      Eigen::Matrix3d R_be;
      Eigen::Vector3d t_be;
      cv::cv2eigen(R_gripper2base[i], R_be);
      cv::cv2eigen(T_gripper2base[i], t_be);
      ceres::CostFunction* closure =
          HandEyeBoardPoseClosureError::Create(R_be, t_be, closure_weight);
      full_problem.AddResidualBlock(
          closure, nullptr, full_ee, full_bb, full_board_poses + i * 6);
    }

    ceres::Solver::Options foptions;
    foptions.linear_solver_type = ceres::SPARSE_SCHUR;
    foptions.minimizer_progress_to_stdout = false;
    foptions.max_num_iterations = 200;
    foptions.num_threads = 8;
    ceres::Solver::Summary fsummary;
    ceres::Solve(foptions, &full_problem, &fsummary);
    std::cout << fsummary.FullReport() << "\n";
    spdlog::info("FULL-BA final cost: {}", fsummary.final_cost);
    if (intrinsics_fx_only)
    {
      full_intr[0] = full_fxy[0];
      full_intr[1] = full_fxy[1];
    }
    spdlog::info(
        "FULL-BA intrinsics: fx={}, fy={}, cx={}, cy={}",
        full_intr[0], full_intr[1], full_intr[2], full_intr[3]);
    spdlog::info(
        "FULL-BA distortion: k1={}, k2={}, p1={}, p2={}, k3={}",
        full_intr[4], full_intr[5], full_intr[6], full_intr[7], full_intr[8]);

    for (int a = 0; a < 6; ++a)
    {
      ba_ee_camera[a] = full_ee[a];
      ba_base_board[a] = full_bb[a];
    }
    for (int a = 0; a < 9; ++a)
      ba_intrinsics[a] = full_intr[a];
    delete[] full_board_poses;
  }

  if (opt_arm_pose)
  {
    spdlog::info("start arm-pose refinement (end-effector poses free with prior) ...");
    const int nviews = R_gripper2base.size();
    double* arm_poses = new double[6 * nviews];
    for (int i = 0; i < nviews; ++i)
    {
      cv::Rodrigues(R_gripper2base[i], rvec);
      arm_poses[i * 6 + 0] = rvec.at<double>(0);
      arm_poses[i * 6 + 1] = rvec.at<double>(1);
      arm_poses[i * 6 + 2] = rvec.at<double>(2);
      arm_poses[i * 6 + 3] = T_gripper2base[i].at<double>(0);
      arm_poses[i * 6 + 4] = T_gripper2base[i].at<double>(1);
      arm_poses[i * 6 + 5] = T_gripper2base[i].at<double>(2);
    }
    double arm_ee[6];
    double arm_bb[6];
    for (int a = 0; a < 6; ++a)
    {
      arm_ee[a] = ba_ee_camera[a];
      arm_bb[a] = ba_base_board[a];
    }

    ceres::Problem arm_problem;
    for (int i = 0; i < nviews; ++i)
    {
      for (int k = 0; k < objs.rows; ++k)
      {
        Eigen::Vector3d obj(
            objs.at<cv::Vec3f>(k)[0],
            objs.at<cv::Vec3f>(k)[1],
            objs.at<cv::Vec3f>(k)[2]);
        Eigen::Vector2d corner(
            all_corners[i].at<cv::Vec2f>(k)[0],
            all_corners[i].at<cv::Vec2f>(k)[1]);
        ceres::CostFunction* reproj =
            HandEyeBoardOnBaseReprojectionErrorArmFixed::Create(
                obj, corner, K_eigen, D_eigen);
        ceres::LossFunction* loss = new ceres::CauchyLoss(1.0);
        arm_problem.AddResidualBlock(
            reproj, loss, arm_ee, arm_bb, arm_poses + i * 6);
      }
    }
    for (int i = 0; i < nviews; ++i)
    {
      Eigen::Matrix3d R_be;
      Eigen::Vector3d t_be;
      cv::cv2eigen(R_gripper2base[i], R_be);
      cv::cv2eigen(T_gripper2base[i], t_be);
      ceres::CostFunction* prior =
          HandEyeArmPosePriorError::Create(R_be, t_be, arm_pose_weight);
      arm_problem.AddResidualBlock(prior, nullptr, arm_poses + i * 6);
    }

    ceres::Solver::Options aoptions;
    aoptions.linear_solver_type = ceres::SPARSE_SCHUR;
    aoptions.minimizer_progress_to_stdout = false;
    aoptions.max_num_iterations = 200;
    aoptions.num_threads = 8;
    ceres::Solver::Summary asummary;
    ceres::Solve(aoptions, &arm_problem, &asummary);
    std::cout << asummary.FullReport() << "\n";
    spdlog::info("ARM-POSE-BA final cost: {}", asummary.final_cost);

    for (int a = 0; a < 6; ++a)
    {
      ba_ee_camera[a] = arm_ee[a];
      ba_base_board[a] = arm_bb[a];
    }
    ba_intrinsics[0] = K.at<float>(0,0);
    ba_intrinsics[1] = K.at<float>(1,1);
    ba_intrinsics[2] = K.at<float>(0,2);
    ba_intrinsics[3] = K.at<float>(1,2);
    ba_intrinsics[4] = D.at<float>(0);
    ba_intrinsics[5] = D.at<float>(1);
    ba_intrinsics[6] = D.at<float>(2);
    ba_intrinsics[7] = D.at<float>(3);
    ba_intrinsics[8] = D.at<float>(4);
    delete[] arm_poses;
  }

  cv::Mat ba_ee_rvec = (cv::Mat_<double>(3, 1) <<
      ba_ee_camera[0], ba_ee_camera[1], ba_ee_camera[2]);
  cv::Mat ba_ee_R;
  cv::Rodrigues(ba_ee_rvec, ba_ee_R);
  cv::Mat ba_ee_t = (cv::Mat_<double>(3, 1) <<
      ba_ee_camera[3], ba_ee_camera[4], ba_ee_camera[5]);
  Homo_cam2gripper =
      calib_eyeinhand::utils::RT2HomogeneousMatrix(ba_ee_R, ba_ee_t);
  calib_eyeinhand::utils::log_cvmat(Homo_cam2gripper, "cam2gripper(COLMAP_BA)");
  calib_eyeinhand::utils::save_cvmat(
      Homo_cam2gripper, "cam2gripper(COLMAP_BA)", outputDir);

  cv::Mat ba_board_rvec = (cv::Mat_<double>(3, 1) <<
      ba_base_board[0], ba_base_board[1], ba_base_board[2]);
  cv::Mat ba_board_R;
  cv::Rodrigues(ba_board_rvec, ba_board_R);
  cv::Mat ba_board_t = (cv::Mat_<double>(3, 1) <<
      ba_base_board[3], ba_base_board[4], ba_base_board[5]);
  cv::Mat Homo_board2base_ba =
      calib_eyeinhand::utils::RT2HomogeneousMatrix(ba_board_R, ba_board_t);
  calib_eyeinhand::utils::log_cvmat(Homo_board2base_ba, "board2base(COLMAP_BA)");
  calib_eyeinhand::utils::save_cvmat(
      Homo_board2base_ba, "board2base(COLMAP_BA)", outputDir);

  // 用优化后的 X 计算闭环指标（target2cam 用检测值）
  HandEyeClosureMetrics ceres_metrics = computeHandEyeClosureMetrics(
      Homo_gripper2base, Homo_cam2gripper, Homo_target2cam);
  spdlog::info(
      "CERES hand-eye closure translation_rmse_m: {}, rotation_rmse_deg: {}",
      ceres_metrics.translation_rmse_m, ceres_metrics.rotation_rmse_deg);
  saveHandEyeClosureMetrics("CERES", ceres_metrics, outputDir);

  // 像素级重投影 RMSE（用优化后的 X、T_base_board 和内参）
  {
    double sq = 0.0;
    int n = 0;
    cv::Mat X_R_obj, t_ee, bb_R, bb_t;
    cv::Rodrigues(ba_ee_rvec, X_R_obj);
    t_ee = ba_ee_t;
    cv::Rodrigues(ba_board_rvec, bb_R);
    bb_t = ba_board_t;
    cv::Mat X_inv_R = X_R_obj.t();
    cv::Mat X_inv_t = -X_inv_R * t_ee;
    cv::Mat bb_inv_R = bb_R.t();
    cv::Mat bb_inv_t = -bb_inv_R * bb_t;
    for (int i = 0; i < R_target2cam.size(); ++i)
    {
      cv::Mat R_be, t_be;
      calib_eyeinhand::utils::HomogeneousMtr2RT(Homo_gripper2base[i], R_be, t_be);
      cv::Mat R_be_inv = R_be.t();
      cv::Mat t_be_inv = -R_be_inv * t_be;
      for (int k = 0; k < objs.rows; ++k)
      {
        cv::Mat p_bd = (cv::Mat_<double>(3, 1) <<
            objs.at<cv::Vec3f>(k)[0],
            objs.at<cv::Vec3f>(k)[1],
            objs.at<cv::Vec3f>(k)[2]);
        cv::Mat p_base = bb_R * p_bd + bb_t;
        cv::Mat p_grip = R_be_inv * (p_base - t_be);
        cv::Mat p_cam = X_inv_R * (p_grip - t_ee);
        double x = p_cam.at<double>(0,0);
        double y = p_cam.at<double>(1,0);
        double z = p_cam.at<double>(2,0);
        double u = x / z, v = y / z;
        double r2 = u*u + v*v, r4 = r2*r2, r6 = r2*r4;
        double k1 = ba_intrinsics[4], k2 = ba_intrinsics[5];
        double p1 = ba_intrinsics[6], p2 = ba_intrinsics[7], k3 = ba_intrinsics[8];
        double radial = 1.0 + k1*r2 + k2*r4 + k3*r6;
        double ud = u*radial + 2.0*p1*u*v + p2*(r2 + 2.0*u*u);
        double vd = v*radial + p1*(r2 + 2.0*v*v) + 2.0*p2*u*v;
        double upx = ba_intrinsics[0]*ud + ba_intrinsics[2];
        double vpx = ba_intrinsics[1]*vd + ba_intrinsics[3];
        double ex = upx - all_corners[i].at<cv::Vec2f>(k)[0];
        double ey = vpx - all_corners[i].at<cv::Vec2f>(k)[1];
        sq += ex*ex + ey*ey;
        ++n;
      }
    }
    spdlog::info(
        "COLMAP-BA reprojection_rmse_px = {:.4f} ({:d} residuals)",
        std::sqrt(sq / n), n);
  }

  // ===== 三结果对比打印：OPENCV_TSAI / TASHAN / CERES =====
  spdlog::info("========== three-method comparison ==========");
  std::vector<MethodComparison> comparison_results = {
      {"OPENCV_TSAI", Homo_cam2gripper_tsai,
       tsa_metrics.translation_rmse_m, tsa_metrics.rotation_rmse_deg,
       "closed-form, raw target2cam"},
      {"TASHAN", Homo_cam2gripper_tashan,
       tashan_metrics.translation_rmse_m, tashan_metrics.rotation_rmse_deg,
       "board-on-base closure, raw target2cam"},
      {"CERES", Homo_cam2gripper,
       ceres_metrics.translation_rmse_m, ceres_metrics.rotation_rmse_deg,
       "COLMAP-style BA, joint intrinsics"},
  };

  for (const auto& item : comparison_results)
  {
    spdlog::info(
        "------ {} ------  [{}]", item.name, item.note);
    calib_eyeinhand::utils::log_cvmat(item.cam2gripper, "cam2gripper");
    spdlog::info(
        "    closure translation_rmse_mm = {:.3f}", item.translation_rmse_m * 1000.0);
    spdlog::info(
        "    closure rotation_rmse_deg  = {:.6f}", item.rotation_rmse_deg);
  }
  spdlog::info("========== end of comparison ==========");

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
