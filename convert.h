#ifndef CONVERT_H
#define CONVERT_H
#include <QImage>
#include <math.h>
#include <visp3/core/vpImageConvert.h>
#include "opencv2/core.hpp"
#include <librealsense2/rs.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visp3/core/vpEigenConversion.h>
#define CALIB_RADIANS 0
#define CALIB_DEGREES 1
using namespace cv;
namespace Convert
{

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
bool isRotationMatrix(Mat &R);
Vec3f rotationMatrixToEulerAngles(Mat &R);
QImage vispToQImage(const vpImage<vpRGBa> &f);
int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou);
int ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou);
int ViSP2Matx(const vpHomogeneousMatrix& visp_in, cv::Matx44d& mat_ou);
cv::Mat qImageToMat(QImage im);
QImage matToQImage(cv::Mat &mat);
void rtToPose(const Mat& R, const Mat& t, Mat& Pose);
void mat2Pcl(const Mat& input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl);
QImage realsenseFrameToQImage(const rs2::frame &f);
void mat2eigen(const Matx44d &mat,Eigen::Affine3f &eigen_pose);
void visp2eigen(const vpHomogeneousMatrix& visp, Eigen::Affine3f &eigen_pose );
bool saveMatFile(const Matx44d &mat, std::string filename);
bool loadMatFile(Matx44d &mat, std::string filename);
bool saveMatFile(const Mat&mat, std::string filename);
bool loadMatFile(Mat &mat, std::string filename);
cv::Mat frame_to_mat(const rs2::frame& f);
cv::Mat depth_frame_to_meters( const rs2::depth_frame & f);
bool pulse2Joint(const std::vector<int32_t> &pulse,std::vector<double> &joint);
bool joint2Pulse(const std::vector<double> &joint,std::vector<int32_t> &pulse);
bool forwardKinematic(const std::vector<double> &joint, Matx44d &pose);
bool inverseKinematic(const Matx44d &pose, std::vector<double> &joint);
}



#endif // CONVERT_H
