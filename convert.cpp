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
    //filename = filename + "/calib.yml";
    FileStorage fs(filename.c_str(), FileStorage::WRITE);
    fs << "calib" << mat;
    fs.release();
    return true;
}

bool Convert::loadMatFile(Matx44d &mat, std::string filename){
    // read
//    filename = filename + "/calib.yml";

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

bool Convert::pulse2Joint(const std::vector<int32_t> &pulse, std::vector<double> &joint)
{
    joint.resize(6);
    if(pulse.size()<6) return false;
    joint[0] = pulse[0]/(34816/30.0)*M_PI/180;
    joint[1] = -pulse[1]/(102400/90.0)*M_PI/180;
    joint[2] = pulse[2]/(51200/90.0)*M_PI/180;
    joint[3] =  -pulse[3]/(10204/30.0)*M_PI/180;
    joint[4] =  pulse[4]/(10204/30.0)*M_PI/180;
    joint[5] =  -pulse[5]/(10204/30.0)*M_PI/180;
    return true;
}

bool Convert::joint2Pulse(const std::vector<double> &joint, std::vector<int32_t> &pulse)
{
    pulse.resize(6);
    if(joint.size()<6) return false;
    pulse[0] = round(joint[0]*(34816/30.0)/(M_PI/180));
    pulse[1] = round(joint[1]*(102400/90.0)/(M_PI/180));
    pulse[2] = round(joint[2]*(51200/90.0)/(M_PI/180));
    pulse[3] = round(joint[3]*(10204/30.0)/(M_PI/180));
    pulse[4] = round(joint[4]*(10204/30.0)/(M_PI/180));
    pulse[5] = round(joint[5]*(10204/30.0)/(M_PI/180));
    return true;
}

bool Convert::forwardKinematic(const std::vector<double> &joint, Matx44d &pose)
{
    if(joint.size()<6) return false;
    double a1 = 0.02;
    double a2 = 0.165;
    double d1 = 0.103;
    double d4 = 0.165;
    double d6 = 0.04;

    double t1 = joint[0];
    double t2 = joint[1]+M_PI/2;
    double t3 = joint[2];
    double t4 = joint[3];
    double t5 = joint[4]-M_PI/2;
    double t6 = joint[5];

//    std::vector<double> t = joint;
//    t[1] = t[1] + M_PI/2;
//    t[4] = t[4] - M_PI/2;
//    double s1 = sin(t[1]);double c1 = cos(t[1]);
//    double s2 = sin(t[2]);double c2 = cos(t[2]);
//    double s3 = sin(t[3]);double c3 = cos(t[3]);
//    double s4 = sin(t[4]);double c4 = cos(t[4]);
//    double s5 = sin(t[5]);double c5 = cos(t[5]);
//    double s0 = sin(t[0]);double c0 = cos(t[0]);
//    double s12 = sin(t[1]+t[2]);double c12 = cos(t[1]+t[2]);
//    Matx44d result{ s5*(c3*s0 + s3*(c0*s1*s2 - c0*c1*c2)) + c5*(c4*(s0*s3 - c3*(c0*s1*s2 - c0*c1*c2)) - s4*(c0*c1*s2 + c0*c2*s1)), c5*(c3*s0 + s3*(c0*s1*s2 - c0*c1*c2)) - s5*(c4*(s0*s3 - c3*(c0*s1*s2 - c0*c1*c2)) - s4*(c0*c1*s2 + c0*c2*s1)), s4*(s0*s3 - c3*(c0*s1*s2 - c0*c1*c2)) + c4*(c0*c1*s2 + c0*c2*s1), a1*c0 + d4*s12*c0 + a2*c0*c1 + d6*s12*c0*c4 + d6*s0*s3*s4 + d6*c0*c1*c2*c3*s4 - d6*c0*c3*s1*s2*s4,
//                    - s5*(c0*c3 - s3*(s0*s1*s2 - c1*c2*s0)) - c5*(c4*(c0*s3 + c3*(s0*s1*s2 - c1*c2*s0)) + s4*(c1*s0*s2 + c2*s0*s1)), s5*(c4*(c0*s3 + c3*(s0*s1*s2 - c1*c2*s0)) + s4*(c1*s0*s2 + c2*s0*s1)) - c5*(c0*c3 - s3*(s0*s1*s2 - c1*c2*s0)), c4*(c1*s0*s2 + c2*s0*s1) - s4*(c0*s3 + c3*(s0*s1*s2 - c1*c2*s0)), a1*s0 + d4*s12*s0 + a2*c1*s0 + d6*s12*c4*s0 - d6*c0*s3*s4 + d6*c1*c2*c3*s0*s4 - d6*c3*s0*s1*s2*s4,
//                    c5*(c12*s4 + s12*c3*c4) - s12*s3*s5,- s5*(c12*s4 + s12*c3*c4) - s12*c5*s3,s12*c3*s4 - c12*c4,d1 - d4*c12 + a2*s1 + (d6*s12*sin(t[3] + t[4]))/2 - d6*c12*c4 - (d6*sin(t[3] - t[4])*s12)/2,
//                    0,0,0,1
//                  };
    Matx44d result{   sin(t6)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + cos(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2))), cos(t6)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2))), sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)), a1*cos(t1) + d4*sin(t2 + t3)*cos(t1) + a2*cos(t1)*cos(t2) + d6*sin(t2 + t3)*cos(t1)*cos(t5) + d6*sin(t1)*sin(t4)*sin(t5) + d6*cos(t1)*cos(t2)*cos(t3)*cos(t4)*sin(t5) - d6*cos(t1)*cos(t4)*sin(t2)*sin(t3)*sin(t5),
     - sin(t6)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) - cos(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))), sin(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2))) - cos(t6)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))), cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))), a1*sin(t1) + d4*sin(t2 + t3)*sin(t1) + a2*cos(t2)*sin(t1) + d6*sin(t2 + t3)*cos(t5)*sin(t1) - d6*cos(t1)*sin(t4)*sin(t5) + d6*cos(t2)*cos(t3)*cos(t4)*sin(t1)*sin(t5) - d6*cos(t4)*sin(t1)*sin(t2)*sin(t3)*sin(t5),
                                                                                                                                                                    cos(t6)*(cos(t2 + t3)*sin(t5) + sin(t2 + t3)*cos(t4)*cos(t5)) - sin(t2 + t3)*sin(t4)*sin(t6),                                                                                                                                                            - sin(t6)*(cos(t2 + t3)*sin(t5) + sin(t2 + t3)*cos(t4)*cos(t5)) - sin(t2 + t3)*cos(t6)*sin(t4),                                                                                                   sin(t2 + t3)*cos(t4)*sin(t5) - cos(t2 + t3)*cos(t5),                                                                                  d1 - d4*cos(t2 + t3) + a2*sin(t2) + (d6*sin(t2 + t3)*sin(t4 + t5))/2 - d6*cos(t2 + t3)*cos(t5) - (d6*sin(t4 - t5)*sin(t2 + t3))/2,
                                                                                                                                                                                                                                                               0,                                                                                                                                                                                                                                                         0,                                                                                                                                                     0,                                                                                                                                                                                                                  1};

    pose = result;
    return  true;
}
bool jointLimit(std::vector<double> joints){
    std::vector<double> joint_limit_up{170*M_PI/180,90*M_PI/180,90*M_PI/180,140*M_PI/180,210*M_PI/180,360*M_PI/180}
                        ,joint_limit_down{-170*M_PI/180,-85*M_PI/180,-50*M_PI/180,-140*M_PI/180,-30*M_PI/180,-360*M_PI/180};
    for (size_t i=0;i<6;i++) {
      if(joints.at(i)>joint_limit_up.at(i))
        return false;
      if(joints.at(i)<joint_limit_down.at(i))
        return false;
    }
    return true;
}
bool Convert::inverseKinematic(const Matx44d &pose, std::vector<double> &joint)
{
    double xc,yc,zc;
      xc = pose(0,3)-0.04*pose(0,2);
      yc = pose(1,3)-0.04*pose(1,2);
      zc = pose(2,3)-0.04*pose(2,2);
      double temp1 = (zc-0.103)*(zc-0.103)-0.165*0.165*2;
      double temp2 = 2*0.165*0.165;

      double temp;
      double c3,s3;
      double c2,s2;
      double temp3;
      double temp4;
      double temp5;
      double theta[6];
      temp= sqrt(xc*xc+yc*yc)-0.02;
      c3 = (temp*temp+temp1)/temp2;
      temp3=0.165*0.165*2+2*0.165*0.165*c3;
      temp4=0.165+0.165*c3;
      joint.resize(6);
      theta[0]=atan2(yc,xc);

      if(c3>=-1&&c3<=1)
      {
        s3=-sqrt(1-c3*c3);
        theta[2]=atan2(s3,c3);
        temp5=0.165*s3;
        c2=(temp*temp4+(zc-0.103)*temp5)/temp3;
        s2=(-temp*temp5+(zc-0.103)*temp4)/temp3;
        if(s2>=-1&&s2<=1&&c2>=-1&&c2<=1)
        {
          theta[1]=atan2(s2,c2);
        }
        else {
          std::cout << "Cannot find IK" << std::endl;
          return false;
        }
      }
      else{
        std::cout << "Cannot find IK" << std::endl;
        return false;
      }

      Matx33d R_,R{pose(0,0),pose(0,1),pose(0,2),
                  pose(1,0),pose(1,1),pose(1,2),
                  pose(2,0),pose(2,1),pose(2,2)
                  };
      double s23 = sin(theta[1] + theta[2]);
      double c23 = cos(theta[1] + theta[2]);
      double c1 = cos(theta[0]);
      double s1 = sin(theta[0]);
      R_(0,0)=-s23*c1;
      R_(0,1)=s1;
      R_(0,2)=c23*c1;

      R_(1,0)=-s23*s1;
      R_(1,1)=-c1;
      R_(1,2)=c23*s1;

      R_(2,0)=c23;
      R_(2,1)=0;
      R_(2,2)=s23;

      R = R_.t()*R;
      theta[3] = atan2(-R(1,2),-R(0,2));
      theta[4] = atan2(-sqrt(R(0,2)*R(0,2)+R(1,2)*R(1,2)),R(2,2));
      theta[5] = atan2(-R(2,1),+R(2,0));

      joint.at(0)=theta[0];
      joint.at(1)=-(theta[1]-M_PI/2);
      joint.at(2) = (theta[2]+M_PI/2);
      joint.at(3)=-theta[3];
      joint.at(4)=(theta[4]+M_PI/2);
      joint.at(5) = -theta[5];
      if(jointLimit(joint)){
        return true;
      }
      else{
        return false;
      }
}

int Convert::Matx2ViSP(vpHomogeneousMatrix &visp_out, const Matx44d &mat_in)
{
    int ret = 0;
    for (int i=0; i<mat_in.rows; i++)
    {
      for (int j=0; j<mat_in.cols; j++)
      {
        visp_out[i][j] = mat_in(i,j);  // new memory is created and data is copied in this line
      }
    }
    return ret;
}
