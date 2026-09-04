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
    bool operator()(const T* const target2cam1, const T* const cam2gripper, const T* const KFxy, T* residuals) const{
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
        T u = p_cam2[0] / p_cam2[2];
        T v = p_cam2[1] / p_cam2[2];

        T fx = KFxy[0];
        T cx = T(K_(0,2));
        T p_x = fx*u + cx;

        T fy = KFxy[1];
        T cy = T(K_(1,2));
        T p_y = fy*v + cy;
        
        residuals[0] = p_x - T(corner_cam2_(0));
        residuals[1] = p_y - T(corner_cam2_(1));


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
        return (new ceres::AutoDiffCostFunction<HandinEyeReprojectionError, 2, 6, 6, 2>(new HandinEyeReprojectionError(obj, corner, K, gr1, gt1, gr2, gt2)));
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


// pairwise i-j 重投影 + 内参(含畸变)可调版本
struct HandinEyeReprojectionErrorIntrinsics
{
    HandinEyeReprojectionErrorIntrinsics(
        const Eigen::Vector3d& obj,
        const Eigen::Vector2d& corner,
        const Eigen::Matrix3d& gr1,
        const Eigen::Vector3d& gt1,
        const Eigen::Matrix3d& gr2,
        const Eigen::Vector3d& gt2)
        : point_target_(obj), corner_cam2_(corner),
          R_g1_(gr1), t_g1_(gt1), R_g2_(gr2), t_g2_(gt2) {}

    template <typename T>
    bool operator()(
        const T* const target2cam1,
        const T* const cam2gripper,
        const T* const intrinsics,
        T* residuals) const
    {
        T p_target[3];
        p_target[0] = T(point_target_(0));
        p_target[1] = T(point_target_(1));
        p_target[2] = T(point_target_(2));

        T p_cam1[3];
        ceres::AngleAxisRotatePoint(target2cam1, p_target, p_cam1);
        p_cam1[0] += target2cam1[3];
        p_cam1[1] += target2cam1[4];
        p_cam1[2] += target2cam1[5];

        T p_gripper1[3];
        ceres::AngleAxisRotatePoint(cam2gripper, p_cam1, p_gripper1);
        p_gripper1[0] += cam2gripper[3];
        p_gripper1[1] += cam2gripper[4];
        p_gripper1[2] += cam2gripper[5];

        Eigen::Matrix<T, 3, 1> p_gripper1_M;
        p_gripper1_M(0) = p_gripper1[0];
        p_gripper1_M(1) = p_gripper1[1];
        p_gripper1_M(2) = p_gripper1[2];
        Eigen::Matrix<T, 3, 1> p_base = R_g1_.cast<T>() * p_gripper1_M + t_g1_.cast<T>();
        Eigen::Matrix<T, 3, 1> p_gripper2_M =
            R_g2_.cast<T>().transpose() * (p_base - t_g2_.cast<T>());
        T p_gripper2[3];
        p_gripper2[0] = p_gripper2_M(0);
        p_gripper2[1] = p_gripper2_M(1);
        p_gripper2[2] = p_gripper2_M(2);

        p_gripper2[0] -= cam2gripper[3];
        p_gripper2[1] -= cam2gripper[4];
        p_gripper2[2] -= cam2gripper[5];
        T p_cam2[3];
        T rotation[3] = {
            T(-1) * cam2gripper[0],
            T(-1) * cam2gripper[1],
            T(-1) * cam2gripper[2]};
        ceres::AngleAxisRotatePoint(rotation, p_gripper2, p_cam2);

        T x = p_cam2[0], y = p_cam2[1], z = p_cam2[2];
        T u = x / z, v = y / z;
        T r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
        T fx = intrinsics[0], fy = intrinsics[1];
        T cx = intrinsics[2], cy = intrinsics[3];
        T k1 = intrinsics[4], k2 = intrinsics[5];
        T p1 = intrinsics[6], p2 = intrinsics[7], k3 = intrinsics[8];
        T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
        T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
        T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;

        residuals[0] = fx * u_d + cx - T(corner_cam2_(0));
        residuals[1] = fy * v_d + cy - T(corner_cam2_(1));
        return true;
    }

    static ceres::CostFunction* Create(
        const Eigen::Vector3d& obj,
        const Eigen::Vector2d& corner,
        const Eigen::Matrix3d& gr1,
        const Eigen::Vector3d& gt1,
        const Eigen::Matrix3d& gr2,
        const Eigen::Vector3d& gt2)
    {
        return new ceres::AutoDiffCostFunction<
            HandinEyeReprojectionErrorIntrinsics, 2, 6, 6, 9>(
            new HandinEyeReprojectionErrorIntrinsics(
                obj, corner, gr1, gt1, gr2, gt2));
    }

    Eigen::Vector3d point_target_;
    Eigen::Vector2d corner_cam2_;
    Eigen::Matrix3d R_g1_;
    Eigen::Vector3d t_g1_;
    Eigen::Matrix3d R_g2_;
    Eigen::Vector3d t_g2_;
};


struct HandEyeBoardOnBaseClosureError
{
    HandEyeBoardOnBaseClosureError(
        const Eigen::Matrix3d& R_base_ee,
        const Eigen::Vector3d& t_base_ee,
        const Eigen::Matrix3d& R_camera_board,
        const Eigen::Vector3d& t_camera_board,
        double scale = 1.0)
        : R_base_ee_(R_base_ee),
          t_base_ee_(t_base_ee),
          R_camera_board_(R_camera_board),
          t_camera_board_(t_camera_board),
          scale_(scale) {}

    // 参数顺序：
    //   ee_camera[6]  = [rotation_vector(3), translation(3)]，表示 T_ee_camera
    //   base_board[6] = [rotation_vector(3), translation(3)]，表示 T_base_board
    template <typename T>
    bool operator()(
        const T* const ee_camera,
        const T* const base_board,
        T* residuals) const
    {
        Eigen::Matrix<T, 3, 3> R_ee_camera;
        ceres::AngleAxisToRotationMatrix(ee_camera, R_ee_camera.data());
        Eigen::Matrix<T, 3, 1> t_ee_camera(
            ee_camera[3], ee_camera[4], ee_camera[5]);

        Eigen::Matrix<T, 3, 3> R_base_board;
        ceres::AngleAxisToRotationMatrix(base_board, R_base_board.data());
        Eigen::Matrix<T, 3, 1> t_base_board(
            base_board[3], base_board[4], base_board[5]);

        const Eigen::Matrix<T, 3, 3> R_base_ee = R_base_ee_.cast<T>();
        const Eigen::Matrix<T, 3, 1> t_base_ee = t_base_ee_.cast<T>();
        const Eigen::Matrix<T, 3, 3> R_camera_board = R_camera_board_.cast<T>();
        const Eigen::Matrix<T, 3, 1> t_camera_board = t_camera_board_.cast<T>();

        // T_base_board_predicted = T_base_ee * T_ee_camera * T_camera_board
        Eigen::Matrix<T, 3, 3> R_pred = R_base_ee * R_ee_camera * R_camera_board;
        Eigen::Matrix<T, 3, 1> t_pred =
            t_base_ee + R_base_ee * (t_ee_camera + R_ee_camera * t_camera_board);

        // delta = T_base_board^-1 * T_base_board_predicted
        Eigen::Matrix<T, 3, 3> R_delta = R_base_board.transpose() * R_pred;
        Eigen::Matrix<T, 3, 1> t_delta =
            R_base_board.transpose() * (t_pred - t_base_board);

        T rvec_delta[3];
        ceres::RotationMatrixToAngleAxis(R_delta.data(), rvec_delta);

        residuals[0] = T(scale_) * t_delta(0);
        residuals[1] = T(scale_) * t_delta(1);
        residuals[2] = T(scale_) * t_delta(2);
        residuals[3] = T(scale_) * rvec_delta[0];
        residuals[4] = T(scale_) * rvec_delta[1];
        residuals[5] = T(scale_) * rvec_delta[2];
        return true;
    }

    static ceres::CostFunction* Create(
        const Eigen::Matrix3d& R_base_ee,
        const Eigen::Vector3d& t_base_ee,
        const Eigen::Matrix3d& R_camera_board,
        const Eigen::Vector3d& t_camera_board,
        double scale = 1.0)
    {
        return new ceres::AutoDiffCostFunction<
            HandEyeBoardOnBaseClosureError, 6, 6, 6>(
            new HandEyeBoardOnBaseClosureError(
                R_base_ee, t_base_ee, R_camera_board, t_camera_board, scale));
    }

    Eigen::Matrix3d R_base_ee_;
    Eigen::Vector3d t_base_ee_;
    Eigen::Matrix3d R_camera_board_;
    Eigen::Vector3d t_camera_board_;
    double scale_;
};


// COLMAP-style 重投影 BA：把棋盘点经已知的 T_base_ee、待求的 T_ee_camera /
// T_base_board 投影回原始图像坐标（含畸变），残差为像素重投影误差。
// 参数顺序：
//   ee_camera[6]  = [rotation_vector(3), translation(3)]，表示 T_ee_camera
//   base_board[6] = [rotation_vector(3), translation(3)]，表示 T_base_board
//   intrinsics[9] = [fx, fy, cx, cy, k1, k2, p1, p2, k3]
struct HandEyeBoardOnBaseReprojectionError
{
  HandEyeBoardOnBaseReprojectionError(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee)
      : point_board_(obj),
        corner_px_(corner),
        R_base_ee_(R_base_ee),
        t_base_ee_(t_base_ee) {}

  template <typename T>
  bool operator()(
      const T* const ee_camera,
      const T* const base_board,
      const T* const intrinsics,
      T* residuals) const
  {
    // T_ee_camera: camera -> gripper
    Eigen::Matrix<T, 3, 3> R_x;
    ceres::AngleAxisToRotationMatrix(ee_camera, R_x.data());
    Eigen::Matrix<T, 3, 1> t_x(
        ee_camera[3], ee_camera[4], ee_camera[5]);

    // T_base_board: board -> base
    Eigen::Matrix<T, 3, 3> R_bb;
    ceres::AngleAxisToRotationMatrix(base_board, R_bb.data());
    Eigen::Matrix<T, 3, 1> t_bb(
        base_board[3], base_board[4], base_board[5]);

    const Eigen::Matrix<T, 3, 3> R_be = R_base_ee_.cast<T>();
    const Eigen::Matrix<T, 3, 1> t_be = t_base_ee_.cast<T>();

    const Eigen::Matrix<T, 3, 1> p_board(
        T(point_board_(0)), T(point_board_(1)), T(point_board_(2)));

    // board -> base
    Eigen::Matrix<T, 3, 1> p_base = R_bb * p_board + t_bb;
    // base -> gripper = inv(T_base_ee) = [R_be^T, -R_be^T * t_be]
    Eigen::Matrix<T, 3, 1> p_gripper =
        R_be.transpose() * (p_base - t_be);
    // gripper -> camera = inv(T_ee_camera) = [R_x^T, -R_x^T * t_x]
    Eigen::Matrix<T, 3, 1> p_cam = R_x.transpose() * (p_gripper - t_x);

    T x = p_cam(0);
    T y = p_cam(1);
    T z = p_cam(2);
    T u = x / z;
    T v = y / z;
    T r2 = u * u + v * v;
    T r4 = r2 * r2;
    T r6 = r4 * r2;

    T fx = intrinsics[0];
    T fy = intrinsics[1];
    T cx = intrinsics[2];
    T cy = intrinsics[3];
    T k1 = intrinsics[4];
    T k2 = intrinsics[5];
    T p1 = intrinsics[6];
    T p2 = intrinsics[7];
    T k3 = intrinsics[8];

    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
    T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;

    T u_px = fx * u_d + cx;
    T v_px = fy * v_d + cy;

    residuals[0] = u_px - T(corner_px_(0));
    residuals[1] = v_px - T(corner_px_(1));
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardOnBaseReprojectionError, 2, 6, 6, 9>(
        new HandEyeBoardOnBaseReprojectionError(
            obj, corner, R_base_ee, t_base_ee));
  }

  Eigen::Vector3d point_board_;
  Eigen::Vector2d corner_px_;
  Eigen::Matrix3d R_base_ee_;
  Eigen::Vector3d t_base_ee_;
};


// 只优化 fx/fy 的轻量重投影（cx/cy 与畸变固定为参考值），用于限制内参自由度
struct HandEyeBoardOnBaseReprojectionErrorFx
{
  HandEyeBoardOnBaseReprojectionErrorFx(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
      : point_board_(obj),
        corner_px_(corner),
        R_base_ee_(R_base_ee),
        t_base_ee_(t_base_ee),
        cx_(K(0, 2)),
        cy_(K(1, 2)),
        k1_(D.size() > 0 ? D(0) : 0.0),
        k2_(D.size() > 1 ? D(1) : 0.0),
        p1_(D.size() > 2 ? D(2) : 0.0),
        p2_(D.size() > 3 ? D(3) : 0.0),
        k3_(D.size() > 4 ? D(4) : 0.0) {}

  template <typename T>
  bool operator()(
      const T* const ee_camera,
      const T* const base_board,
      const T* const fxy,
      T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_x;
    ceres::AngleAxisToRotationMatrix(ee_camera, R_x.data());
    Eigen::Matrix<T, 3, 1> t_x(
        ee_camera[3], ee_camera[4], ee_camera[5]);
    Eigen::Matrix<T, 3, 3> R_bb;
    ceres::AngleAxisToRotationMatrix(base_board, R_bb.data());
    Eigen::Matrix<T, 3, 1> t_bb(
        base_board[3], base_board[4], base_board[5]);

    const Eigen::Matrix<T, 3, 3> R_be = R_base_ee_.cast<T>();
    const Eigen::Matrix<T, 3, 1> t_be = t_base_ee_.cast<T>();
    const Eigen::Matrix<T, 3, 1> p_board(
        T(point_board_(0)), T(point_board_(1)), T(point_board_(2)));

    Eigen::Matrix<T, 3, 1> p_base = R_bb * p_board + t_bb;
    Eigen::Matrix<T, 3, 1> p_gripper =
        R_be.transpose() * (p_base - t_be);
    Eigen::Matrix<T, 3, 1> p_cam = R_x.transpose() * (p_gripper - t_x);

    T x = p_cam(0), y = p_cam(1), z = p_cam(2);
    T u = x / z, v = y / z;
    T r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    T k1 = T(k1_), k2 = T(k2_), k3 = T(k3_);
    T p1 = T(p1_), p2 = T(p2_);
    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
    T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;

    T u_px = fxy[0] * u_d + T(cx_);
    T v_px = fxy[1] * v_d + T(cy_);
    residuals[0] = u_px - T(corner_px_(0));
    residuals[1] = v_px - T(corner_px_(1));
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardOnBaseReprojectionErrorFx, 2, 6, 6, 2>(
        new HandEyeBoardOnBaseReprojectionErrorFx(
            obj, corner, R_base_ee, t_base_ee, K, D));
  }

  Eigen::Vector3d point_board_;
  Eigen::Vector2d corner_px_;
  Eigen::Matrix3d R_base_ee_;
  Eigen::Vector3d t_base_ee_;
  double cx_, cy_;
  double k1_, k2_, p1_, p2_, k3_;
};


// 全 BA：棋盘点通过“某张图的板位姿”直接投影（板位姿为自由参数）
struct HandEyeBoardPoseReprojectionError
{
  HandEyeBoardPoseReprojectionError(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner)
      : point_board_(obj), corner_px_(corner) {}

  template <typename T>
  bool operator()(
      const T* const board_pose,
      const T* const intrinsics,
      T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_cb;
    ceres::AngleAxisToRotationMatrix(board_pose, R_cb.data());
    Eigen::Matrix<T, 3, 1> t_cb(
        board_pose[3], board_pose[4], board_pose[5]);
    Eigen::Matrix<T, 3, 1> p_board(
        T(point_board_(0)), T(point_board_(1)), T(point_board_(2)));
    Eigen::Matrix<T, 3, 1> p_cam = R_cb * p_board + t_cb;

    T x = p_cam(0), y = p_cam(1), z = p_cam(2);
    T u = x / z, v = y / z;
    T r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    T fx = intrinsics[0], fy = intrinsics[1];
    T cx = intrinsics[2], cy = intrinsics[3];
    T k1 = intrinsics[4], k2 = intrinsics[5];
    T p1 = intrinsics[6], p2 = intrinsics[7], k3 = intrinsics[8];
    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
    T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;
    residuals[0] = fx * u_d + cx - T(corner_px_(0));
    residuals[1] = fy * v_d + cy - T(corner_px_(1));
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardPoseReprojectionError, 2, 6, 9>(
        new HandEyeBoardPoseReprojectionError(obj, corner));
  }

  Eigen::Vector3d point_board_;
  Eigen::Vector2d corner_px_;
};


// 全 BA（板位姿自由）但只优化 fx/fy，cx/cy 与畸变固定
struct HandEyeBoardPoseReprojectionErrorFx
{
  HandEyeBoardPoseReprojectionErrorFx(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
      : point_board_(obj),
        corner_px_(corner),
        cx_(K(0, 2)),
        cy_(K(1, 2)),
        k1_(D.size() > 0 ? D(0) : 0.0),
        k2_(D.size() > 1 ? D(1) : 0.0),
        p1_(D.size() > 2 ? D(2) : 0.0),
        p2_(D.size() > 3 ? D(3) : 0.0),
        k3_(D.size() > 4 ? D(4) : 0.0) {}

  template <typename T>
  bool operator()(
      const T* const board_pose,
      const T* const fxy,
      T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_cb;
    ceres::AngleAxisToRotationMatrix(board_pose, R_cb.data());
    Eigen::Matrix<T, 3, 1> t_cb(
        board_pose[3], board_pose[4], board_pose[5]);
    Eigen::Matrix<T, 3, 1> p_board(
        T(point_board_(0)), T(point_board_(1)), T(point_board_(2)));
    Eigen::Matrix<T, 3, 1> p_cam = R_cb * p_board + t_cb;

    T x = p_cam(0), y = p_cam(1), z = p_cam(2);
    T u = x / z, v = y / z;
    T r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    T k1 = T(k1_), k2 = T(k2_), k3 = T(k3_);
    T p1 = T(p1_), p2 = T(p2_);
    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
    T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;
    residuals[0] = fxy[0] * u_d + T(cx_) - T(corner_px_(0));
    residuals[1] = fxy[1] * v_d + T(cy_) - T(corner_px_(1));
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardPoseReprojectionErrorFx, 2, 6, 2>(
        new HandEyeBoardPoseReprojectionErrorFx(obj, corner, K, D));
  }

  Eigen::Vector3d point_board_;
  Eigen::Vector2d corner_px_;
  double cx_, cy_;
  double k1_, k2_, p1_, p2_, k3_;
};


// 全 BA（板位姿自由）且内参完全固定
struct HandEyeBoardPoseReprojectionErrorFixed
{
  HandEyeBoardPoseReprojectionErrorFixed(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
      : point_board_(obj),
        corner_px_(corner),
        fx_(K(0, 0)),
        fy_(K(1, 1)),
        cx_(K(0, 2)),
        cy_(K(1, 2)),
        k1_(D.size() > 0 ? D(0) : 0.0),
        k2_(D.size() > 1 ? D(1) : 0.0),
        p1_(D.size() > 2 ? D(2) : 0.0),
        p2_(D.size() > 3 ? D(3) : 0.0),
        k3_(D.size() > 4 ? D(4) : 0.0) {}

  template <typename T>
  bool operator()(const T* const board_pose, T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_cb;
    ceres::AngleAxisToRotationMatrix(board_pose, R_cb.data());
    Eigen::Matrix<T, 3, 1> t_cb(
        board_pose[3], board_pose[4], board_pose[5]);
    Eigen::Matrix<T, 3, 1> p_board(
        T(point_board_(0)), T(point_board_(1)), T(point_board_(2)));
    Eigen::Matrix<T, 3, 1> p_cam = R_cb * p_board + t_cb;

    T x = p_cam(0), y = p_cam(1), z = p_cam(2);
    T u = x / z, v = y / z;
    T r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    T k1 = T(k1_), k2 = T(k2_), k3 = T(k3_);
    T p1 = T(p1_), p2 = T(p2_);
    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
    T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;
    residuals[0] = T(fx_) * u_d + T(cx_) - T(corner_px_(0));
    residuals[1] = T(fy_) * v_d + T(cy_) - T(corner_px_(1));
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardPoseReprojectionErrorFixed, 2, 6>(
        new HandEyeBoardPoseReprojectionErrorFixed(obj, corner, K, D));
  }

  Eigen::Vector3d point_board_;
  Eigen::Vector2d corner_px_;
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, p1_, p2_, k3_;
};


// 全 BA：手眼闭环，其中某张图的板位姿是自由参数
struct HandEyeBoardPoseClosureError
{
  HandEyeBoardPoseClosureError(
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee,
      double scale = 1.0)
      : R_base_ee_(R_base_ee),
        t_base_ee_(t_base_ee),
        scale_(scale) {}

  template <typename T>
  bool operator()(
      const T* const ee_camera,
      const T* const base_board,
      const T* const board_pose,
      T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_x;
    ceres::AngleAxisToRotationMatrix(ee_camera, R_x.data());
    Eigen::Matrix<T, 3, 1> t_x(ee_camera[3], ee_camera[4], ee_camera[5]);
    Eigen::Matrix<T, 3, 3> R_bb;
    ceres::AngleAxisToRotationMatrix(base_board, R_bb.data());
    Eigen::Matrix<T, 3, 1> t_bb(
        base_board[3], base_board[4], base_board[5]);
    Eigen::Matrix<T, 3, 3> R_cb;
    ceres::AngleAxisToRotationMatrix(board_pose, R_cb.data());
    Eigen::Matrix<T, 3, 1> t_cb(
        board_pose[3], board_pose[4], board_pose[5]);

    const Eigen::Matrix<T, 3, 3> R_be = R_base_ee_.cast<T>();
    const Eigen::Matrix<T, 3, 1> t_be = t_base_ee_.cast<T>();

    Eigen::Matrix<T, 3, 3> R_pred = R_be * R_x * R_cb;
    Eigen::Matrix<T, 3, 1> t_pred =
        t_be + R_be * (t_x + R_x * t_cb);
    Eigen::Matrix<T, 3, 3> R_delta = R_bb.transpose() * R_pred;
    Eigen::Matrix<T, 3, 1> t_delta =
        R_bb.transpose() * (t_pred - t_bb);
    T rvec_delta[3];
    ceres::RotationMatrixToAngleAxis(R_delta.data(), rvec_delta);

    residuals[0] = T(scale_) * t_delta(0);
    residuals[1] = T(scale_) * t_delta(1);
    residuals[2] = T(scale_) * t_delta(2);
    residuals[3] = T(scale_) * rvec_delta[0];
    residuals[4] = T(scale_) * rvec_delta[1];
    residuals[5] = T(scale_) * rvec_delta[2];
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee,
      double scale = 1.0)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardPoseClosureError, 6, 6, 6, 6>(
        new HandEyeBoardPoseClosureError(R_base_ee, t_base_ee, scale));
  }

  Eigen::Matrix3d R_base_ee_;
  Eigen::Vector3d t_base_ee_;
  double scale_;
};


// 末端位姿可调 + 内参固定的重投影：
//   board -> base(T_base_board) -> gripper(inv T_base_ee) -> camera(inv T_ee_camera) -> 内参
struct HandEyeBoardOnBaseReprojectionErrorArmFixed
{
  HandEyeBoardOnBaseReprojectionErrorArmFixed(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
      : point_board_(obj),
        corner_px_(corner),
        fx_(K(0, 0)),
        fy_(K(1, 1)),
        cx_(K(0, 2)),
        cy_(K(1, 2)),
        k1_(D.size() > 0 ? D(0) : 0.0),
        k2_(D.size() > 1 ? D(1) : 0.0),
        p1_(D.size() > 2 ? D(2) : 0.0),
        p2_(D.size() > 3 ? D(3) : 0.0),
        k3_(D.size() > 4 ? D(4) : 0.0) {}

  template <typename T>
  bool operator()(
      const T* const ee_camera,
      const T* const base_board,
      const T* const base_ee,
      T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_x;
    ceres::AngleAxisToRotationMatrix(ee_camera, R_x.data());
    Eigen::Matrix<T, 3, 1> t_x(ee_camera[3], ee_camera[4], ee_camera[5]);
    Eigen::Matrix<T, 3, 3> R_bb;
    ceres::AngleAxisToRotationMatrix(base_board, R_bb.data());
    Eigen::Matrix<T, 3, 1> t_bb(
        base_board[3], base_board[4], base_board[5]);
    Eigen::Matrix<T, 3, 3> R_be;
    ceres::AngleAxisToRotationMatrix(base_ee, R_be.data());
    Eigen::Matrix<T, 3, 1> t_be(base_ee[3], base_ee[4], base_ee[5]);

    Eigen::Matrix<T, 3, 1> p_board(
        T(point_board_(0)), T(point_board_(1)), T(point_board_(2)));
    Eigen::Matrix<T, 3, 1> p_base = R_bb * p_board + t_bb;
    Eigen::Matrix<T, 3, 1> p_gripper = R_be.transpose() * (p_base - t_be);
    Eigen::Matrix<T, 3, 1> p_cam = R_x.transpose() * (p_gripper - t_x);

    T x = p_cam(0), y = p_cam(1), z = p_cam(2);
    T u = x / z, v = y / z;
    T r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    T k1 = T(k1_), k2 = T(k2_), k3 = T(k3_);
    T p1 = T(p1_), p2 = T(p2_);
    T radial = T(1.0) + k1 * r2 + k2 * r4 + k3 * r6;
    T u_d = u * radial + T(2.0) * p1 * u * v + p2 * (r2 + T(2.0) * u * u);
    T v_d = v * radial + p1 * (r2 + T(2.0) * v * v) + T(2.0) * p2 * u * v;
    residuals[0] = T(fx_) * u_d + T(cx_) - T(corner_px_(0));
    residuals[1] = T(fy_) * v_d + T(cy_) - T(corner_px_(1));
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& obj,
      const Eigen::Vector2d& corner,
      const Eigen::Matrix3d& K,
      const Eigen::VectorXd& D)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeBoardOnBaseReprojectionErrorArmFixed, 2, 6, 6, 6>(
        new HandEyeBoardOnBaseReprojectionErrorArmFixed(obj, corner, K, D));
  }

  Eigen::Vector3d point_board_;
  Eigen::Vector2d corner_px_;
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, p1_, p2_, k3_;
};


// 末端位姿先验：优化后的 T_base_ee 偏离机械臂上报值的 SE3 残差
struct HandEyeArmPosePriorError
{
  HandEyeArmPosePriorError(
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee,
      double scale = 1.0)
      : R_base_ee_(R_base_ee),
        t_base_ee_(t_base_ee),
        scale_(scale) {}

  template <typename T>
  bool operator()(const T* const base_ee, T* residuals) const
  {
    Eigen::Matrix<T, 3, 3> R_be;
    ceres::AngleAxisToRotationMatrix(base_ee, R_be.data());
    Eigen::Matrix<T, 3, 1> t_be(base_ee[3], base_ee[4], base_ee[5]);
    const Eigen::Matrix<T, 3, 3> R_meas = R_base_ee_.cast<T>();
    const Eigen::Matrix<T, 3, 1> t_meas = t_base_ee_.cast<T>();
    Eigen::Matrix<T, 3, 3> R_delta = R_meas.transpose() * R_be;
    Eigen::Matrix<T, 3, 1> t_delta =
        R_meas.transpose() * (t_be - t_meas);
    T rvec_delta[3];
    ceres::RotationMatrixToAngleAxis(R_delta.data(), rvec_delta);
    residuals[0] = T(scale_) * t_delta(0);
    residuals[1] = T(scale_) * t_delta(1);
    residuals[2] = T(scale_) * t_delta(2);
    residuals[3] = T(scale_) * rvec_delta[0];
    residuals[4] = T(scale_) * rvec_delta[1];
    residuals[5] = T(scale_) * rvec_delta[2];
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Matrix3d& R_base_ee,
      const Eigen::Vector3d& t_base_ee,
      double scale = 1.0)
  {
    return new ceres::AutoDiffCostFunction<
        HandEyeArmPosePriorError, 6, 6>(
        new HandEyeArmPosePriorError(R_base_ee, t_base_ee, scale));
  }

  Eigen::Matrix3d R_base_ee_;
  Eigen::Vector3d t_base_ee_;
  double scale_;
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


        residuals[0] = distance * T(_plane1(0))/normal_norm*T(1000);
        residuals[1] = distance * T(_plane1(1))/normal_norm*T(1000);
        residuals[2] = distance * T(_plane1(2))/normal_norm*T(1000);

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
 
