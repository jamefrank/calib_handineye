#include "utils.h"
#include "spdlog/spdlog.h"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>

bool calib_eyeinhand::utils::parse_K(const YAML::Node & config, std::string name, cv::Mat & K)
{
    if(config[name]){
        if(config[name].IsSequence()){
            int num = config[name].size();
            if(9==num || 3==num){
                std::vector<float> tmp;   
                if(9 == num){
                    for(int i=0; i<num; i++){
                        float value = config[name][i].as<float>();
                        tmp.push_back(value);
                    }
                }
                if(3 == num){
                    for(int i=0; i<num; i++){
                        for (int j = 0; j < config[name][i].size(); ++j) {
                            float value = config[name][i][j].as<float>();
                            tmp.push_back(value);
                        }
                    }
                }
                K = cv::Mat(3, 3, CV_32F, tmp.data()).clone();
                return true;
            }
            else
                spdlog::error("3x3 or 9 supported");
        }
        else
            spdlog::error("[{0}] not list", name);
    }
    else{
        spdlog::error("{0} not exists", name);
    }
    
    return false;
}

void calib_eyeinhand::utils::parse_D(const YAML::Node & config, std::string name, cv::Mat & D_)
{
    std::vector<float> D;   //TODO
    for (std::size_t i = 0; i < config["D"].size(); ++i) {
        float value = config["D"][i].as<float>();
        D.push_back(value);
    }
    D_ = cv::Mat(1, D.size(), CV_32F, D.data()).clone();
}

void calib_eyeinhand::utils::log_cvmat(const cv::Mat & mat, const std::string & name)
{
    std::ostringstream oss;
    oss << name << " = " << std::endl << mat << std::endl;
    spdlog::info("{}", oss.str());
}

cv::Mat calib_eyeinhand::utils::load_arm_pose(const std::string & file_path)
{
    // 
    cv::Mat data_mat = cv::Mat::zeros(0, 6, CV_64F);

    std::ifstream file(file_path);
    assert(file.is_open());

    double x, y, z, rx, ry, rz;
    while (file >> x >> y >> z >> rx >> ry >> rz) {
		x /= 1000.0f;
		y /= 1000.0f;
		z /= 1000.0f;
		rx *= M_PI/180.0f;
		ry *= M_PI/180.0f;
		rz *= M_PI/180.0f;
        cv::Mat row = (cv::Mat_<double>(1, 6) << x, y, z, rx, ry, rz);
        data_mat.push_back(row);
    }

    file.close();

    return data_mat;
}

cv::Mat calib_eyeinhand::utils::attitudeVectorToMatrix(const cv::Mat & m, bool useQuaternion, const std::string & seq)
{
CV_Assert(m.total() == 6 || m.total() == 10);
	//if (m.cols == 1)	//转置矩阵为行矩阵
	//	m = m.t();	

	cv::Mat temp = cv::Mat::eye(4, 4, CV_64FC1);

	if (useQuaternion)
	{
		cv::Vec4d quaternionVec = m({ 3,0,4,1 });   //读取存储的四元数
		quaternionToRotatedMatrix(quaternionVec).copyTo(temp({0,0,3,3}));  
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
		{
			rotVec = m({ 3,0,3,1 });   //读取存储的欧拉角
		}
		if (m.total() == 10)
		{
			rotVec = m({ 7,0,3,1 });
		}
		//如果seq为空，表示传入的是3*1旋转向量，否则，传入的是欧拉角
		if (0 == seq.compare(""))
		{
			cv::Rodrigues(rotVec, temp({ 0,0,3,3 }));   //罗德利斯转换
		}
		else
		{
			eulerAngleToRotateMatrix(rotVec, seq).copyTo(temp({ 0,0,3,3 }));
		}
	}
	//存入平移矩阵
	temp({ 3,0,1,3 }) = m({ 0,0,3,1 }).t();
	return temp;   //返回转换结束的齐次矩阵
}

cv::Mat calib_eyeinhand::utils::quaternionToRotatedMatrix(const cv::Vec4d & q)
{
	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	double q0q0 = q0 * q0 , q1q1 = q1 * q1 , q2q2 = q2 * q2, q3q3 = q3 * q3;
	double q0q1 = q0 * q1 , q0q2 = q0 * q2 , q0q3 = q0 * q3;
	double q1q2 = q1 * q2, q1q3 = q1 * q3;
	double q2q3 = q2 * q3;
	//根据公式得来
	cv::Mat RotMtr = (cv::Mat_<double>(3, 3) << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
		2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
		2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3));
	//这种形式等价
	/*Mat RotMtr = (Mat_<double>(3, 3) << (1 - 2 * (q2q2 + q3q3)), 2 * (q1q2 - q0q3), 2 * (q1q3 + q0q2),
										 2 * (q1q2 + q0q3), 1 - 2 * (q1q1 + q3q3), 2 * (q2q3 - q0q1),
										 2 * (q1q3 - q0q2), 2 * (q2q3 + q0q1), (1 - 2 * (q1q1 + q2q2)));*/

	return RotMtr;
}

cv::Mat calib_eyeinhand::utils::eulerAngleToRotateMatrix(const cv::Mat & eulerAngle, const std::string & seq)
{
    CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);//检查参数是否正确

	cv::Matx13d m(eulerAngle);				//<double, 1, 3>

	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto rxs = sin(rx), rxc = cos(rx);
	auto rys = sin(ry), ryc = cos(ry);
	auto rzs = sin(rz), rzc = cos(rz);

	//XYZ方向的旋转矩阵
	cv::Mat RotX = (cv::Mat_<double>(3, 3) << 1, 0, 0,
		0, rxc, -rxs,
		0, rxs, rxc);
	cv::Mat RotY = (cv::Mat_<double>(3, 3) << ryc, 0, rys,
		0,	  1, 0,
		-rys, 0, ryc);
	cv::Mat RotZ = (cv::Mat_<double>(3, 3) << rzc, -rzs, 0,
		rzs, rzc, 0,
		0, 0, 1);
	//按顺序合成后的旋转矩阵
	cv::Mat rotMat;

	if (seq == "zyx") rotMat = RotX * RotY * RotZ;
	else if (seq == "yzx") rotMat = RotX * RotZ * RotY;
	else if (seq == "zxy") rotMat = RotY * RotX * RotZ;
	else if (seq == "yxz") rotMat = RotZ * RotX * RotY;
	else if (seq == "xyz") rotMat = RotZ * RotY * RotX;
	else if (seq == "xzy") rotMat = RotY * RotZ * RotX;
	else
	{
        spdlog::error("Euler Angle Sequence string is wrong...");
        exit(-1);
	}
	if (!isRotatedMatrix(rotMat))		//欧拉角特殊情况下会出现死锁
	{
        spdlog::error("Euler Angle convert to RotatedMatrix failed...");
		exit(-1);
	}
	return rotMat;
}

bool calib_eyeinhand::utils::isRotatedMatrix(cv::Mat & R)
{
	cv::Mat temp33 = R({ 0,0,3,3 });	//无论输入是几阶矩阵，均提取它的三阶矩阵
	cv::Mat Rt;
	transpose(temp33, Rt);  //转置矩阵
	cv::Mat shouldBeIdentity = Rt * temp33;//是旋转矩阵则乘积为单位矩阵
	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return cv::norm(I, shouldBeIdentity) < 1e-6;
}

void calib_eyeinhand::utils::HomogeneousMtr2RT(cv::Mat & HomoMtr, cv::Mat & R, cv::Mat & T)
{
    //Mat R_HomoMtr = HomoMtr(Rect(0, 0, 3, 3)); //注意Rect取值
	//Mat T_HomoMtr = HomoMtr(Rect(3, 0, 1, 3));
	//R_HomoMtr.copyTo(R);
	//T_HomoMtr.copyTo(T);
	/*HomoMtr(Rect(0, 0, 3, 3)).copyTo(R);
	HomoMtr(Rect(3, 0, 1, 3)).copyTo(T);*/
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = HomoMtr(R_rect);
	T = HomoMtr(T_rect);
}

cv::Mat calib_eyeinhand::utils::RT2HomogeneousMatrix(const cv::Mat & R, const cv::Mat & T)
{
	cv::Mat HomoMtr;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << 
										R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
										R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
										R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
										0, 0, 0);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
										T.at<double>(0,0),
										T.at<double>(1,0),
										T.at<double>(2,0),
										1);
	cv::hconcat(R1, T1, HomoMtr);		//矩阵拼接
	return HomoMtr;
}

cv::Mat calib_eyeinhand::utils::genObjs(const cv::Size & boardSize, float squareSize)
{
	float squareLength = squareSize / 1000.0f;
	cv::Mat objs(boardSize.width*boardSize.height,1,CV_32FC3);
	for(int i=0;i<objs.rows;i++)
	{
		int idx = i % boardSize.width;
		int idy = i / boardSize.width;
		objs.at<cv::Vec3f>(i)[0] =(idx + 1) * squareLength;
		objs.at<cv::Vec3f>(i)[1] =(idy + 1) * squareLength;
		objs.at<cv::Vec3f>(i)[2] =0;
	}
	return objs;
}

bool calib_eyeinhand::utils::detectCharucoCornersAndPose(
	const cv::Size& boardSize,
	float squareSize, float markerSize,
	const cv::Mat& image, const cv::Mat& K, const cv::Mat& D, const cv::Mat& objs,
	cv::Mat& imageCopy, cv::Mat& charucoCorners, cv::Mat& charucoIds, cv::Mat& rvec, cv::Mat& tvec, double& rerror
)
{
	int squaresX = boardSize.width + 1;
	int squaresY = boardSize.height + 1;
	float squareLength = squareSize / 1000.0f;
	float markerLength = markerSize / 1000.0f;
	cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	cv::Ptr<cv::aruco::CharucoBoard> board_ = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary_);

	cv::Mat gray, undistImg;
	cv::undistort(image, undistImg, K, D);
    undistImg.copyTo(imageCopy);
    cv::cvtColor(undistImg, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);
	// cv::Mat blur_usm;
    // cv::GaussianBlur(gray, blur_usm, cv::Size(0, 0), 25);
    // cv::addWeighted(gray, 1.5, blur_usm, -0.5, 0, gray);

	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, marker_rejected;

    cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, parameters, marker_rejected);

    cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray, board_, charucoCorners, charucoIds);
    if(charucoIds.rows == boardSize.width * boardSize.height){
        
        bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board_, K, cv::Mat(), rvec, tvec);
        if(valid){
            // calc error
            cv::Mat imgpoints;
            cv::projectPoints(objs, rvec, tvec, K, cv::Mat(), imgpoints);
            rerror = 0;
            for(int i=0;i<imgpoints.rows;i++)
            {
                cv::Vec2f e = imgpoints.at<cv::Vec2f>(i) - charucoCorners.at<cv::Vec2f>(i);
                rerror += std::sqrt(e[0]*e[0]+e[1]*e[1]);
            }
            rerror /= imgpoints.rows;
            
            //
			cv::aruco::drawDetectedMarkers(imageCopy, marker_corners, marker_ids);
			cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
			cv::drawFrameAxes(imageCopy, K, cv::Mat(), rvec, tvec, 0.1f);
			return true;
        }
    }
  
	return false;
}

bool calib_eyeinhand::utils::detectChessCornersAndPose(const cv::Size & boardSize, const cv::Mat & image, const cv::Mat & K, const cv::Mat & D, const cv::Mat & objs, cv::Mat & imageCopy, cv::Mat & corners, cv::Mat & rvec, cv::Mat & tvec, double & rerror)
{
	cv::Mat gray, undistImg;
	cv::undistort(image, undistImg, K, D);
    undistImg.copyTo(imageCopy);
    cv::cvtColor(undistImg, gray, cv::COLOR_BGR2GRAY);

	bool success = cv::findChessboardCorners(gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
	if(success && corners.rows==boardSize.width*boardSize.height){
		cv::cornerSubPix(gray, corners, cv::Size(3,3), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		bool valid = cv::solvePnP(objs, corners, K, cv::Mat(), rvec, tvec);
		if(valid){
			// calc error
            cv::Mat imgpoints;
            cv::projectPoints(objs, rvec, tvec, K, cv::Mat(), imgpoints);
            rerror = 0;
            for(int i=0;i<imgpoints.rows;i++)
            {
                cv::Vec2f e = imgpoints.at<cv::Vec2f>(i) - corners.at<cv::Vec2f>(i);
                rerror += std::sqrt(e[0]*e[0]+e[1]*e[1]);
            }
            rerror /= imgpoints.rows;

			cv::drawChessboardCorners(imageCopy, boardSize, corners, success);
			cv::drawFrameAxes(imageCopy, K, cv::Mat(), rvec, tvec, 0.1f);
			return true;
		}
	}

	return false;
}

