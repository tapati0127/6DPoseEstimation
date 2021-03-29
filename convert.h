#ifndef CONVERT_H
#define CONVERT_H
#include <QImage>
#include <visp3/core/vpImageConvert.h>
#include "opencv2/core.hpp"
#include <librealsense2/rs.hpp>
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
};


#endif // CONVERT_H
