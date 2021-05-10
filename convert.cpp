#include "convert.h"


Mat Convert::eulerAnglesToRotationMatrix(Vec3f &theta)
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

bool Convert::isRotationMatrix(Mat &R)
{
    Mat Rt;
            transpose(R, Rt);
            Mat shouldBeIdentity = Rt * R;
            Mat I = Mat::eye(3,3, shouldBeIdentity.type());

            return  norm(I, shouldBeIdentity) < 1e-6;
}

Vec3f Convert::rotationMatrixToEulerAngles(Mat &R)
{
    assert(Convert::isRotationMatrix(R));

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

QImage Convert::vispToQImage(const vpImage<vpRGBa> &f)
{
    auto r = QImage(reinterpret_cast<unsigned char *>(f.bitmap), f.getWidth(), f.getHeight(), f.getWidth()*4, QImage::Format_RGBA8888);
    return r;
}

int Convert::Mat2ViSP(const Mat &mat_in, vpHomogeneousMatrix &visp_ou)
{
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

int Convert::ViSP2Mat(const vpHomogeneousMatrix &visp_in, Mat &mat_ou)
{
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

Mat Convert::qImageToMat(QImage im)
{
    QImage tmp = im.convertToFormat(QImage::Format::Format_RGB888);
    cv::Mat mat(tmp.height(),tmp.width(),CV_8UC3,tmp.bits());
    return mat;
}

QImage Convert::matToQImage(Mat &mat)
{
    return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format::Format_RGB888);
}


void Convert::rtToPose(const Mat &R, const Mat &t, Mat &Pose)
{
    Matx34f P;
          hconcat(R, t, P);
          vconcat(P, Matx14f(0, 0, 0, 1), Pose);
}

void Convert::mat2Pcl(const Mat &input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl)
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

QImage Convert::realsenseFrameToQImage(const rs2::frame &f)
{
    using namespace rs2;

            auto vf = f.as<video_frame>();
            const int w = vf.get_width();
            const int h = vf.get_height();
            //std::cout << "w " << w << "h " << h << std::endl;

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


void Convert::visp2eigen(const vpHomogeneousMatrix &visp, Eigen::Affine3f &eigen_pose)
{
    Eigen::Matrix4d matrix_pose;
    vp::visp2eigen(visp,matrix_pose);
    eigen_pose.matrix() = matrix_pose.cast<float>();
}

void Convert::mat2eigen(const Matx44d &pose, Eigen::Affine3f &eigen_pose)
{
    vpHomogeneousMatrix temp;
    Mat dmat(pose);
    Mat mat;
    dmat.convertTo(mat,CV_32F);
    Mat2ViSP(mat,temp);
    visp2eigen(temp,eigen_pose);
}

bool Convert::saveMatFile(const Matx44d &mat, std::string filename){
    // write
    filename = filename + "/calib.yml";
    FileStorage fs(filename.c_str(), FileStorage::WRITE);
    fs << "calib" << mat;
    fs.release();
    return true;
}

bool Convert::loadMatFile(Matx44d &mat, std::string filename){
    // read
    filename = filename + "/calib.yml";
    FileStorage fs(filename.c_str(), FileStorage::READ);
    fs["calib"] >> mat;
    fs.release();
    return true;
}

int Convert::ViSP2Matx(const vpHomogeneousMatrix &visp_in, Matx44d &mat_ou)
{
    cv::Mat mat,matd;
    ViSP2Mat(visp_in,mat);
    mat.convertTo(matd,CV_64F);
    Matx44d temp(matd);
    mat_ou = temp;
}

bool Convert::saveMatFile(const Mat &mat, std::string filename)
{
    // write
    FileStorage fs(filename.c_str(), FileStorage::WRITE);
    fs << "calib" << mat;
    fs.release();
    return true;
}

bool Convert::loadMatFile(Mat &mat, std::string filename)
{
    // read
    FileStorage fs(filename.c_str(), FileStorage::READ);
    fs["calib"] >> mat;
    fs.release();
    return true;
}
cv::Mat Convert::frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat Convert::depth_frame_to_meters( const rs2::depth_frame & f )
{
    cv::Mat dm = frame_to_mat(f);
    dm.convertTo( dm, CV_64F );
    dm = dm * f.get_units();
    return dm;
}
