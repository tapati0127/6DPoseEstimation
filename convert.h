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
class  Convert{
public:
    // Calculates rotation matrix given euler angles.
    static Mat eulerAnglesToRotationMatrix(Vec3f &theta)
    {
        // Calculate rotation about x axis
        Mat R_x = (Mat_<float>(3,3) <<
                   1,       0,              0,
                   0,       cos(theta[0]),   -sin(theta[0]),
                   0,       sin(theta[0]),   cos(theta[0])
                   );

        // Calculate rotation about y axis
        Mat R_y = (Mat_<float>(3,3) <<
                   cos(theta[1]),    0,      sin(theta[1]),
                   0,               1,      0,
                   -sin(theta[1]),   0,      cos(theta[1])
                   );

        // Calculate rotation about z axis
        Mat R_z = (Mat_<float>(3,3) <<
                   cos(theta[2]),    -sin(theta[2]),      0,
                   sin(theta[2]),    cos(theta[2]),       0,
                   0,               0,                  1);


        // Combined rotation matrix
        Mat R = R_z * R_y * R_x;
        //Mat R = R_x * R_y * R_z;
        return R;

    }

    // Checks if a matrix is a valid rotation matrix.
    static bool isRotationMatrix(Mat &R)
    {
        Mat Rt;
        transpose(R, Rt);
        Mat shouldBeIdentity = Rt * R;
        Mat I = Mat::eye(3,3, shouldBeIdentity.type());

        return  norm(I, shouldBeIdentity) < 1e-6;

    }

    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).
    static Vec3f rotationMatrixToEulerAngles(Mat &R)
    {

        assert(isRotationMatrix(R));

        float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

        bool singular = sy < 1e-6; // If

        float x, y, z;
        if (!singular)
        {
            x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
            y = atan2(-R.at<float>(2,0), sy);
            z = atan2(R.at<float>(1,0), R.at<float>(0,0));
        }
        else
        {
            x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
            y = atan2(-R.at<float>(2,0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);



    }

    static QImage vispToQImage(const vpImage<vpRGBa> &f)
    {
        auto r = QImage(reinterpret_cast<unsigned char *>(f.bitmap), f.getWidth(), f.getHeight(), f.getWidth()*4, QImage::Format_RGBA8888);
        return r;
    }
    static QImage realsenseFrameToQImage(const rs2::frame &f)
    {
        using namespace rs2;

        auto vf = f.as<video_frame>();
        const int w = vf.get_width();
        const int h = vf.get_height();

        if (f.get_profile().format() == RS2_FORMAT_RGB8)
        {
            auto r = QImage((uchar*) f.get_data(), w, h, w*3, QImage::Format_RGB888);
            return r;
        }
        else if (f.get_profile().format() == RS2_FORMAT_Z16)
        {
            // only if you have Qt > 5.13
            auto r = QImage((uchar*) f.get_data(), w, h, w*2, QImage::Format_Grayscale8);
            return r;
        }

        throw std::runtime_error("Frame format is not supported yet!");
    }
    static int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou){
        int ret = 0;

        //if (mat_in.type() != CV_32FC1 || mat_in.type() != CV_64FC1)
        if (mat_in.type() != CV_32FC1)
        {
          //std::cout << "[HandEyeCalib] Mat input is not floating-point number!" << std::endl;
          std::cout << "[HandEyeCalib] Mat input is not double floating-point number!" << std::endl;
          ret = 1;
          return ret;
        }

        for (int i=0; i<mat_in.rows; i++)
        {
          for (int j=0; j<mat_in.cols; j++)
          {
            visp_ou[i][j] = mat_in.ptr<float>(i)[j];  // new memory is created and data is copied in this line
          }
        }

        return ret;
    }
    static int ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou){
        int ret = 0;

        mat_ou = cv::Mat::zeros(visp_in.getRows(),visp_in.getCols(),CV_32FC1);

        for (int i=0; i<mat_ou.rows; i++)
        {
          for (int j=0; j<mat_ou.cols; j++)
          {
            mat_ou.ptr<float>(i)[j] = visp_in[i][j];  // new memory is created and data is copied in this line
          }
        }

        return ret;
    }
    static cv::Mat qImageToMat(QImage im){
        QImage tmp = im.convertToFormat(QImage::Format::Format_RGB888);
        cv::Mat mat(tmp.height(),tmp.width(),CV_8UC3,tmp.bits());
        return mat;
    }
    static QImage matToQImage(cv::Mat &mat) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format::Format_RGB888);
    }
    static void Euler(const cv::Mat& src, cv::Mat& dst, int argType)
    {
        if((src.rows == 3) && (src.cols == 3))
        {
            //convert rotation matrix to 3 angles (pitch, yaw, roll)
            dst = cv::Mat(3, 1, CV_32F);
            double pitch, yaw, roll;

            if(src.at<double>(0,2) < -0.998)
            {
                pitch = -atan2(src.at<double>(1,0), src.at<double>(1,1));
                yaw = -M_PI_2;
                roll = 0.;
            }
            else if(src.at<double>(0,2) > 0.998)
            {
                pitch = atan2(src.at<double>(1,0), src.at<double>(1,1));
                yaw = M_PI_2;
                roll = 0.;
            }
            else
            {
                pitch = atan2(-src.at<double>(1,2), src.at<double>(2,2));
                yaw = asin(src.at<double>(0,2));
                roll = atan2(-src.at<double>(0,1), src.at<double>(0,0));
            }

            if(argType == CALIB_DEGREES)
            {
                pitch *= 180./M_PI;
                yaw *= 180./M_PI;
                roll *= 180./M_PI;
            }
            else if(argType != CALIB_RADIANS)
                CV_Error(cv::Error::StsBadFlag, "Invalid argument type");

            dst.at<double>(0,0) = pitch;
            dst.at<double>(1,0) = yaw;
            dst.at<double>(2,0) = roll;
        }
        else if( (src.cols == 1 && src.rows == 3) ||
                 (src.cols == 3 && src.rows == 1 ) )
        {
            //convert vector which contains 3 angles (pitch, yaw, roll) to rotation matrix
            float pitch, yaw, roll;
            if(src.cols == 1 && src.rows == 3)
            {
                pitch = src.at<float>(0,0);
                yaw = src.at<float>(1,0);
                roll = src.at<float>(2,0);
            }
            else{
                pitch = src.at<float>(0,0);
                yaw = src.at<float>(0,1);
                roll = src.at<float>(0,2);
            }

            if(argType == CALIB_DEGREES)
            {
                pitch *= M_PI / 180.;
                yaw *= M_PI / 180.;
                roll *= M_PI / 180.;
            }
            else if(argType != CALIB_RADIANS)
                CV_Error(cv::Error::StsBadFlag, "Invalid argument type");

            dst = cv::Mat(3, 3, CV_32F);
            cv::Mat M(3, 3, CV_32F);
            cv::Mat i = cv::Mat::eye(3, 3, CV_32F);
            i.copyTo(dst);
            i.copyTo(M);

            float* pR = dst.ptr<float>();
            pR[4] = cos(pitch);
            pR[7] = sin(pitch);
            pR[8] = pR[4];
            pR[5] = -pR[7];

            float* pM = M.ptr<float>();
            pM[0] = cos(yaw);
            pM[2] = sin(yaw);
            pM[8] = pM[0];
            pM[6] = -pM[2];

            dst *= M;
            i.copyTo(M);
            pM[0] = cos(roll);
            pM[3] = sin(roll);
            pM[4] = pM[0];
            pM[1] = -pM[3];

            dst *= M;
        }
        else
            CV_Error(cv::Error::StsBadFlag, "Input matrix must be 1x3, 3x1 or 3x3" );
    }
    static void rtToPose(const Mat& R, const Mat& t, Mat& Pose)
    {
      Matx34f P;
      hconcat(R, t, P);
      vconcat(P, Matx14f(0, 0, 0, 1), Pose);
    }
    static void mat2Pcl(const Mat& input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl)
    {
        pcl->points.resize(input.rows);
        for (int i=0;i<input.rows;i++) {
            pcl->points.at(i).x =input.at<float>(i,0);
            pcl->points.at(i).y =input.at<float>(i,1);
            pcl->points.at(i).z =input.at<float>(i,2);
            pcl->points.at(i).r =0;
            pcl->points.at(i).g =0;
            pcl->points.at(i).b =255;
        }
    }


};


#endif // CONVERT_H
