#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Core>



struct HandinEyeReprojectionError
{
    HandinEyeReprojectionError(
        const Eigen::Vector3d& obj, 
        const Eigen::Vector2d& corner, 
        const Eigen::Matrix3d& K,
        const Eigen::Matrix3d& gr1,
        const Eigen::Vector3d& gt1,
        const Eigen::Matrix3d& gr2,
        const Eigen::Vector3d& gt2
        )
        : point_target_(obj), corner_cam2_(corner), K_(K), R_g1_(gr1), t_g1_(gt1), R_g2_(gr2), t_g2_(gt2) {}

    // target2cam1,cam2gripper: rvec+tvec
    template <typename T>
    bool operator()(const T* const target2cam1, const T* const cam2gripper, T* residuals) const{
        //
        T p_target[3];
        p_target[0] = T(point_target_(0));
        p_target[1] = T(point_target_(1));
        p_target[2] = T(point_target_(2));
        // 
        T p_cam1[3];
        ceres::AngleAxisRotatePoint(target2cam1, p_target, p_cam1);
        p_cam1[0] += target2cam1[3];
        p_cam1[1] += target2cam1[4];
        p_cam1[2] += target2cam1[5];
        //
        T p_gripper1[3];
        ceres::AngleAxisRotatePoint(cam2gripper, p_cam1, p_gripper1);
        p_gripper1[0] += cam2gripper[3];
        p_gripper1[1] += cam2gripper[4];
        p_gripper1[2] += cam2gripper[5];
        Eigen::Matrix<T, 3, 1> p_gripper1_M;
        p_gripper1_M(0) = p_gripper1[0];
        p_gripper1_M(1) = p_gripper1[1];
        p_gripper1_M(2) = p_gripper1[2];
        // 
        Eigen::Matrix<T, 3, 1> p_base = R_g1_.cast<T>()*p_gripper1_M + t_g1_.cast<T>();
        //
        Eigen::Matrix<T, 3, 1> p_gripper2_M = R_g2_.cast<T>().transpose() * (p_base - t_g2_.cast<T>());
        T p_gripper2[3];
        p_gripper2[0] = p_gripper2_M(0);
        p_gripper2[1] = p_gripper2_M(1);
        p_gripper2[2] = p_gripper2_M(2);
        //
        p_gripper2[0] -= cam2gripper[3];
        p_gripper2[1] -= cam2gripper[4];
        p_gripper2[2] -= cam2gripper[5];
        T p_cam2[3];
        T rotation[3] = {T(-1) * cam2gripper[0], T(-1) * cam2gripper[1], T(-1) * cam2gripper[2]};
        ceres::AngleAxisRotatePoint(rotation, p_gripper2, p_cam2);

        // 
        T p_cam2_Obs[2];
        T fx = T(K_(0,0));
        T cx = T(K_(0,2));
        T fy = T(K_(1,1));
        T cy = T(K_(1,2));

        p_cam2_Obs[0] = (T(corner_cam2_(0)) - cx)/fx*p_cam2[2];
        p_cam2_Obs[1] = (T(corner_cam2_(1)) - cy)/fy*p_cam2[2];

        //
        residuals[0] = p_cam2[0] - p_cam2_Obs[0];
        residuals[1] = p_cam2[1] - p_cam2_Obs[1];

        return true;
    }

    static ceres::CostFunction *Create(
        const Eigen::Vector3d& obj, 
        const Eigen::Vector2d& corner, 
        const Eigen::Matrix3d& K,
        const Eigen::Matrix3d& gr1,
        const Eigen::Vector3d& gt1,
        const Eigen::Matrix3d& gr2,
        const Eigen::Vector3d& gt2)
    {
        return (new ceres::AutoDiffCostFunction<HandinEyeReprojectionError, 2, 6, 6>(new HandinEyeReprojectionError(obj, corner, K, gr1, gt1, gr2, gt2)));
    }

    // members
    Eigen::Vector3d point_target_;
    Eigen::Vector2d corner_cam2_;
    Eigen::Matrix3d K_;
    Eigen::Matrix3d R_g1_;
    Eigen::Matrix3d R_g2_;
    Eigen::Vector3d t_g1_;
    Eigen::Vector3d t_g2_;
};


class Point2PlaneError {
public:
    Point2PlaneError(Eigen::Vector4d plane1, Eigen::Vector3d point2, bool binverse) :
        _plane1(plane1), _point2(point2), _binverse(binverse), normal_norm_(plane1.head<3>().norm()) {}
    // pose 为 旋转向量+平移向量
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        T point[3] = { T(_point2(0)), T(_point2(1)), T(_point2(2)) };
        T predict[3];
        //计算对称投影误差
        if (_binverse) {
            // inverse
            T t_inverse[3];
            t_inverse[0] = point[0] - pose[3];
            t_inverse[1] = point[1] - pose[4];
            t_inverse[2] = point[2] - pose[5];
            T rotation[3] = { T(-1) * pose[0], T(-1) * pose[1], T(-1) * pose[2] };
            ceres::AngleAxisRotatePoint(rotation, t_inverse, predict);
        }
        else {
            ceres::AngleAxisRotatePoint(pose, point, predict);
            predict[0] += pose[3];
            predict[1] += pose[4];
            predict[2] += pose[5];
        }
        
        T normal_norm = T(normal_norm_);
        T distance = (predict[0] * T(_plane1(0)) + predict[1] * T(_plane1(1)) + predict[2] * T(_plane1(2)) + T(_plane1(3))) / normal_norm;


        residuals[0] = distance * T(_plane1(0))/normal_norm;
        residuals[1] = distance * T(_plane1(1))/normal_norm;
        residuals[2] = distance * T(_plane1(2))/normal_norm;

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector4d& plane1, const Eigen::Vector3d& point2,
        const bool& binverse) {
        return (
            new ceres::AutoDiffCostFunction<Point2PlaneError, 3, 6>(new Point2PlaneError(plane1, point2, binverse)));
    }

    Eigen::Vector4d _plane1;
    Eigen::Vector3d _point2;
    double normal_norm_;
    bool _binverse;
};
 


