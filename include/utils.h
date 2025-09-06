#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace calib_eyeinhand{
    namespace utils{
        bool parse_K(const YAML::Node& config, std::string name, cv::Mat& K);
        void parse_D(const YAML::Node& config, std::string name, cv::Mat& D);
        void log_cvmat(const cv::Mat& mat, const std::string& name = "Mat");

        bool loadPointCloud(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        cv::Mat load_arm_pose(const std::string& file_path);

        cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq);
        cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q); // w,x,y,z
        cv::Mat eulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const std::string& seq);
        bool isRotatedMatrix(cv::Mat& R);
        void HomogeneousMtr2RT(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T);
        cv::Mat RT2HomogeneousMatrix(const cv::Mat& R,const cv::Mat& T);

        cv::Mat genObjs(const cv::Size& boardSize, float squareSize);
        bool detectCharucoCornersAndPose(
            const cv::Size& boardSize,
            float squareSize, float markerSize,
            const cv::Mat& img, const cv::Mat& K, const cv::Mat& D, const cv::Mat& objs,
            cv::Mat& draw_img, cv::Mat& charucoCorners, cv::Mat& charucoIds, cv::Mat& rvec, cv::Mat& tvec, double& rerror
        );
        bool detectChessCornersAndPose(
            const cv::Size& boardSize,
            const cv::Mat& img, const cv::Mat& K, const cv::Mat& D, const cv::Mat& objs,
            cv::Mat& draw_img, cv::Mat& corners,cv::Mat& rvec, cv::Mat& tvec, double& rerror
        );

        //提取平面，并且将内点映射到提取的平面上
        void extractPlaneAndProjectiton(const pcl::PointCloud<pcl::PointXYZ>::Ptr frame, Eigen::Vector4d& coef, pcl::PointCloud<pcl::PointXYZ>::Ptr plane);
    }
}





