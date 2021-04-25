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

#define CALIB_RADIANS 0
#define CALIB_DEGREES 1
using namespace cv;

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
bool isRotationMatrix(Mat &R);
Vec3f rotationMatrixToEulerAngles(Mat &R);
QImage vispToQImage(const vpImage<vpRGBa> &f);
int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou);
int ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou);
cv::Mat qImageToMat(QImage im);
QImage matToQImage(cv::Mat &mat);
void rtToPose(const Mat& R, const Mat& t, Mat& Pose);
void mat2Pcl(const Mat& input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl);
QImage realsenseFrameToQImage(const rs2::frame &f);



#endif // CONVERT_H
