#include "handeyecalibration.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include <opencv2/calib3d.hpp>
#include <unistd.h>
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpEigenConversion.h>
HandEyeCalibration::HandEyeCalibration()
{   
    qRegisterMetaType<Eigen::Affine3f>();
    motoudp = new MotoUDP(QHostAddress("192.168.1.11"),10040);
    motoudp->ConnectMotoman();
    //motoudp->TurnOnServo();
    //connect(motoudp,&MotoUDP::receivePosition,this,&HandEyeCalibration::receivedPosition);
}

void HandEyeCalibration::run()
{
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.063;
  float quad_decimate = 1.0;
  int nThreads = 1;
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  bool display_off = false;
  try {
       std::cout << "Use Realsense 2 grabber" << std::endl;
       vpRealSense2 g;
       rs2::config config;
       unsigned int width = 640, height = 480;
       config.disable_stream(RS2_STREAM_DEPTH);
       config.disable_stream(RS2_STREAM_INFRARED);
       config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_RGBA8, 30);
       config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Z16, 30);

       vpImage<unsigned char> I;
       vpImage<vpRGBa> I_color(height, width);
       vpImage<uint16_t> I_depth_raw(height, width);
       vpImage<vpRGBa> I_depth;

       g.open(config);
       const float depth_scale = g.getDepthScale();
       rs2::align align_to_color = RS2_STREAM_COLOR;
       g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                 nullptr, nullptr, &align_to_color);

       std::cout << "Read camera parameters from Realsense device" << std::endl;
       vpCameraParameters cam;
       cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);

       std::cout << cam << std::endl;
       std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
       std::cout << "tagFamily: " << tagFamily << std::endl;
       std::cout << "nThreads : " << nThreads << std::endl;
       std::cout << "Z aligned: " << align_frame << std::endl;

       vpImage<vpRGBa> I_color2 = I_color;
       vpImage<float> depthMap;
       vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

       vpDisplay *d1 = NULL;
       vpDisplay *d2 = NULL;
       vpDisplay *d3 = NULL;
       if (! display_off) {
   #ifdef VISP_HAVE_X11
         d1 = new vpDisplayX(I_color, 100, 30, "Pose from Homography");
         d2 = new vpDisplayX(I_color2, I_color.getWidth()+120, 30, "Pose from RGBD fusion");
         d3 = new vpDisplayX(I_depth, 100, I_color.getHeight()+70, "Depth");
   #elif defined(VISP_HAVE_GDI)
         d1 = new vpDisplayGDI(I_color, 100, 30, "Pose from Homography");
         d2 = new vpDisplayGDI(I_color2, I_color.getWidth()+120, 30, "Pose from RGBD fusion");
         d3 = new vpDisplayGDI(I_depth, 100, I_color.getHeight()+70, "Depth");
   #elif defined(VISP_HAVE_OPENCV)
         d1 = new vpDisplayOpenCV(I_color, 100, 30, "Pose from Homography");
         d2 = new vpDisplayOpenCV(I_color2, I_color.getWidth()+120, 30, "Pose from RGBD fusion");
         d3 = new vpDisplayOpenCV(I_depth, 100, I_color.getHeight()+70, "Depth");
   #endif
       }

       vpDetectorAprilTag detector(tagFamily);

       detector.setAprilTagQuadDecimate(quad_decimate);
       detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
       detector.setAprilTagNbThreads(nThreads);
       detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
       detector.setZAlignedWithCameraAxis(align_frame);

       while (true) {
         g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                   nullptr, nullptr, &align_to_color);
         QImage tmp;
         tmp = Convert::vispToQImage(I_color);
         Q_EMIT framesReady(tmp,tmp);

         vpImageConvert::convert(I_color, I);
         I_color2 = I_color;
         vpImageConvert::convert(I_color, I);
         vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

         vpDisplay::display(I_color);
         vpDisplay::display(I_color2);
         vpDisplay::display(I_depth);

         depthMap.resize(I_depth_raw.getHeight(), I_depth_raw.getWidth());
         for (unsigned int i = 0; i < I_depth_raw.getHeight(); i++) {
             for (unsigned int j = 0; j < I_depth_raw.getWidth(); j++) {
                 if (I_depth_raw[i][j]) {
                     float Z = I_depth_raw[i][j] * depth_scale;
                     depthMap[i][j] = Z;
                 } else {
                     depthMap[i][j] = 0;
                 }
             }
         }

         vpDisplay::display(I_color);
         vpDisplay::display(I_color2);
         vpDisplay::display(I_depth);


         if(trigger){
             std::cout << "Finding..." << std::endl;

             std::vector<vpHomogeneousMatrix> cMo_vec;
             detector.detect(I, tagSize, cam, cMo_vec);

             // Display camera pose for each tag
             for (size_t i = 0; i < cMo_vec.size(); i++) {
               vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
             }

             std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
             std::vector<int> tags_id = detector.getTagsId();
             std::map<int, double> tags_size;
             tags_size[-1] = tagSize; // Default tag size
             std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);

             for (size_t i = 0; i < tags_corners.size(); i++) {
               vpHomogeneousMatrix cMo;
               double confidence_index;
               if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo, &confidence_index)) {
                     if (confidence_index > 0.5) {
                         cv::Mat test;
                         vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::none, 3);
                         Convert::ViSP2Mat(cMo,test);
                         std::cout << " Found cam2target pose " << std::endl << test << std::endl;
                         int32_t pos[6];
                         if(motoudp->GetPosition(pos)){
                             receivedPosition(pos);
                             R_target2cam.push_back(test.rowRange(0,3).colRange(0,3));
                             t_target2cam.push_back(test.rowRange(0,3).col(3));
                             pose_target2cam.push_back(cMo);
                             std::cout << " Added cam2target pose "<< std::endl;
                             isReceivePositionFromRobot = false;
                         }
                     }
                     else {
                        std::cout << "Fail" << std::endl;
                     }

               }
             }
             std::cout << "End finding" << std::endl;
             trigger=false;
         }
         vpDisplay::flush(I_color);
         vpDisplay::flush(I_color2);
         vpDisplay::flush(I_depth);
       }

     } catch (const vpException &e) {
       std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

}

void HandEyeCalibration::receivedPosition(int32_t*pos)
{

    float position[6];
    position[0]=float(*pos)/1000000;
    position[1]=float(*(pos+1))/1000000;
    position[2]=float(*(pos+2))/1000000;
    position[3]=float(*(pos+3))/10000*M_PI/180;
    position[4]=float(*(pos+4))/10000*M_PI/180;
    position[5]=float(*(pos+5))/10000*M_PI/180;
    cv::Mat position_vector = cv::Mat(3,1, CV_32F, position);
    cv::Mat temp = cv::Mat(3,1, CV_32F, position+3);
    //std::cout << "pos " << temp << "vec" << position_vector << std::endl;
    cv::Mat rotation_matrix;
    cv::Mat pose(4,4, CV_32F);
    Convert::Euler(temp,rotation_matrix,CALIB_RADIANS);
    Convert::rtToPose(rotation_matrix,position_vector,pose);
    pose = pose.inv();
    R_base2gripper.push_back(pose.colRange(0,3).rowRange(0,3));
    t_base2gripper.push_back(pose.col(3).rowRange(0,3));
    vpHomogeneousMatrix pose_;
    Convert::Mat2ViSP(pose,pose_);
    pose_base2gripper.push_back(pose_);
    std::cout << "gripper2base pose " << std::endl << pose << std::endl;
    isReceivePositionFromRobot = true;
}

void HandEyeCalibration::caculatePose()
{
    if(R_target2cam.size()>=3&&t_target2cam.size()>=3){
       cv::calibrateHandEye(R_base2gripper,t_base2gripper,R_target2cam,t_target2cam,R_cam2base,t_cam2base);
       int ret = vpHandEyeCalibration::calibrate(pose_base2gripper, pose_target2cam, pose_cam2base);
       std::cout << "***OPENCV***" << std::endl;
       std::cout << R_cam2base << std::endl;
       std::cout << t_cam2base << std::endl << std::endl;
       if(ret==0){
           std::cout << "***VISP***" << std::endl;
           std::cout << pose_cam2base << std::endl;
           Eigen::Affine3f eigen_pose_cam2base;
           Eigen::Matrix4d matrix_pose_cam2base;
           vp::visp2eigen(pose_cam2base,matrix_pose_cam2base);
           eigen_pose_cam2base.matrix() = matrix_pose_cam2base.cast<float>();
           Q_EMIT finishCalibrate(eigen_pose_cam2base);

       }
       else{
           std::cout << "Fail VISP" << std::endl;
       }


       /*for(size_t i=0;i<R_target2cam.size();i++){
           cv::Matx44d target2cam,cam2gripper,gripper2base,pose;
           Convert::rtToPose(R_cam2gripper,t_cam2gripper,cam2gripper);
           Convert::rtToPose(R_target2cam.at(i),t_target2cam.at(i),target2cam);
           Convert::rtToPose(R_gripper2base.at(i),t_gripper2base.at(i),gripper2base);
           pose = target2cam*cam2gripper*gripper2base;
           std::cout << "***cam2base***" << std::endl;
           std::cout << pose << std::endl;
           std::cout << pose << std::endl << std::endl;
       }*/

    }
    else{
        std::cout << "Please get more pose" << std::endl;
    }
}

void HandEyeCalibration::test()
{
    vpHomogeneousMatrix pose_cam2base_,pose_cam2base_true(1.2,-1.3,1.4,-M_PI_4,M_PI_4,-M_PI_4),
            pose_gripper2object_true(0.2,0.3,0.4,-M_PI/9,M_PI_4/2,-M_PI_4/3);
    std::cout << "pose_cam2base_true " << std::endl << pose_cam2base_true << std::endl;
    std::cout << "pose_gripper2object_true " << std::endl << pose_gripper2object_true << std::endl;
    vpHomogeneousMatrix base2gripper(1,-1,1,-M_PI_4/2,M_PI_4/5,-M_PI_4/7);
    pose_base2gripper.push_back(base2gripper);
    pose_target2cam.push_back(pose_gripper2object_true.inverse()*base2gripper.inverse()*pose_cam2base_true.inverse());

    vpHomogeneousMatrix base2gripper1(0,-1,1,M_PI_4/3,M_PI_4/7,M_PI_4/2);
    pose_base2gripper.push_back(base2gripper1);
    pose_target2cam.push_back(pose_gripper2object_true.inverse()*base2gripper1.inverse()*pose_cam2base_true.inverse());

    vpHomogeneousMatrix base2gripper2(0,1.2,-3,M_PI_2/3,-M_PI_4/7,-M_PI_2/5);
    pose_base2gripper.push_back(base2gripper2);
    pose_target2cam.push_back(pose_gripper2object_true.inverse()*base2gripper2.inverse()*pose_cam2base_true.inverse());

    vpHomogeneousMatrix base2gripper3(-0.9,-1.2,-0.1,M_PI_2/9,M_PI_4/15,M_PI_2/25);
    pose_base2gripper.push_back(base2gripper3);
    pose_target2cam.push_back(pose_gripper2object_true.inverse()*base2gripper3.inverse()*pose_cam2base_true.inverse());

    vpHomogeneousMatrix base2gripper4(1.9,0.2,5.1,M_PI_2/1,-M_PI_4/4,-M_PI_2/3);
    pose_base2gripper.push_back(base2gripper4);
    pose_target2cam.push_back(pose_gripper2object_true.inverse()*base2gripper4.inverse()*pose_cam2base_true.inverse());

    vpHomogeneousMatrix base2gripper5(2,1.12,0.1,-M_PI_2/4,-M_PI_4/5,-M_PI_2/25);
    pose_base2gripper.push_back(base2gripper5);
    pose_target2cam.push_back(pose_gripper2object_true.inverse()*base2gripper5.inverse()*pose_cam2base_true.inverse());

    if(vpHandEyeCalibration::calibrate(pose_base2gripper,pose_target2cam,pose_cam2base_)==0){
        std::cout << " Result " << pose_cam2base_ << std::endl;
    }
    else{
        std::cout << " Fail " << std::endl;
    }

}




