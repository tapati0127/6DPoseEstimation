#include "handeyecalibration.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include <opencv2/calib3d.hpp>
#include <unistd.h>
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpEigenConversion.h>
HandEyeCalibration::HandEyeCalibration(const QString &ip,std::string cameraFile)
{   
    this->cameraFile=cameraFile;
    qRegisterMetaType<Eigen::Affine3f>();
    motoudp = new MotoUDP(QHostAddress(ip),10040);
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
       rs2::context ctx;
       auto list = ctx.query_devices();
       if(list.size() > 0){

           g.open(config);
           auto pro = g.getPipeline().get_active_profile();
           rs2::device dev = pro.get_device();
           auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
           std::ifstream file(cameraFile);
           std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
           advanced_mode_dev.load_json(str);


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

           while (!thread_stop) {
             g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                       nullptr, nullptr, &align_to_color);
             QImage tmp;
             tmp = vispToQImage(I_color);
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



                 //std::cout << "Finding..." << std::endl;

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
                   vpHomogeneousMatrix cMo_;
                   double confidence_index;
                   if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo_, &confidence_index)) {
                         if (confidence_index > 0.5) {
                             cv::Mat test;
                             vpDisplay::displayFrame(I_color2, cMo_, cam, tagSize/2, vpColor::none, 3);
                             ViSP2Mat(cMo_,test);
                             if(trigger){
                             int32_t pos[6];
                             if(motoudp->GetPosition(pos)){
                                 receivedPosition(pos);
                                 R_target2cam.push_back(test.rowRange(0,3).colRange(0,3));
                                 t_target2cam.push_back(test.rowRange(0,3).col(3));

//                                 if(t_target2cam.size()>=2){
//                                     std::cout << "Translation " << norm(t_target2cam.at(t_target2cam.size()-1)-t_target2cam.at(t_target2cam.size()-2));
//                                 }

                                 //pose_target2cam.push_back(cMo);
                                 this->cMo.push_back(cMo_);
                                 std::cout << "Added cMo_ pose "<< std::endl<< cMo_ << std::endl;
                                 isReceivePositionFromRobot = false;
                             }
                             else{
                                 std::cout << "Cannot connect to controller, please check again! " << std::endl;
                             }
                             trigger=false;
                             }
                         }
                         else {
                            std::cout << "Fail" << std::endl;
                         }

                   }
                 }
             vpDisplay::flush(I_color);
             vpDisplay::flush(I_color2);
             vpDisplay::flush(I_depth);
           }
       }
       else{
           std::cout << "Cannot connect with camera, simulate" << std::endl;
           while(!thread_stop){
               //std::cout << "Test Thread" << std::endl;
               if(trigger){
                   trigger = false;
                   int32_t pos[6];
                   if(motoudp->GetPosition(pos)){
                       receivedPosition(pos);}
                   else{
                       std::cout << "Check connection with robot" << std::endl;
                   }
               }

           }
       }

     } catch (const vpException &e) {
       std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

}

void HandEyeCalibration::stop()
{
    thread_stop=true;
    this->wait(700);
    //g.close();
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
    cv::Vec3f temp(position[3],position[4],position[5]);
    std::cout << "ori " << temp*180/M_PI << std::endl;
    cv::Mat rotation_matrix;
    cv::Mat pose(4,4, CV_32F);
    rotation_matrix = eulerAnglesToRotationMatrix(temp);
    //std::cout << rotation_matrix*180.0/M_PI << std::endl;
    rtToPose(rotation_matrix,position_vector,pose);
    //std::cout << "rotation_matrix " << std::endl << rotation_matrix << std::endl;

    pose = pose.inv();
    R_base2gripper.push_back(pose.colRange(0,3).rowRange(0,3));
    t_base2gripper.push_back(pose.col(3).rowRange(0,3));

//    if(t_base2gripper.size()>=2){
//        std::cout << "Translation " << norm(t_base2gripper.at(t_base2gripper.size()-1)-t_base2gripper.at(t_base2gripper.size()-2));
//    }

    vpHomogeneousMatrix pose_;
    Mat2ViSP(pose,pose_);

    eMw.push_back(pose_);
    std::cout << "eMw pose " << std::endl << pose << std::endl;
    isReceivePositionFromRobot = true;

//    Eigen::Affine3f eigen_pose;
//    Eigen::Matrix4d matrix_pose;
//    vp::visp2eigen(pose_.inverse(),matrix_pose);
//    eigen_pose.matrix() = matrix_pose.cast<float>();
//    Q_EMIT finishCalibrate(eigen_pose);
}

void HandEyeCalibration::caculatePose()
{
    if(eMw.size()>=3&&cMo.size()>=3){
       cv::calibrateHandEye(R_base2gripper,t_base2gripper,R_target2cam,t_target2cam,R_cam2base,t_cam2base,cv::CALIB_HAND_EYE_PARK);
       //wMc.eye();
       int ret = vpHandEyeCalibration::calibrate(cMo,eMw,wMc);
       std::cout << "***OPENCV***" << std::endl;
       std::cout << R_cam2base << std::endl;
       std::cout << t_cam2base << std::endl << std::endl;
       if(ret==0){
           std::cout << "***VISP***" << std::endl;
           std::cout << wMc << std::endl;
           Eigen::Affine3f eigen_pose_cam2base;
           Eigen::Matrix4d matrix_pose_cam2base;
           vp::visp2eigen(wMc,matrix_pose_cam2base);
           eigen_pose_cam2base.matrix() = matrix_pose_cam2base.cast<float>();
           Q_EMIT finishCalibrate(eigen_pose_cam2base);
       }
       else{
           std::cout << "Fail VISP" << std::endl;
       }
    }
    else{
        std::cout << "Please get more pose" << std::endl;
    }
}

void HandEyeCalibration::test()
{
    vpHomogeneousMatrix wMc_,wMc_true(1.2,-1.3,1.4,-M_PI_4,M_PI_4,-M_PI_4),
            oMe_true(0.2,0.3,0.4,-M_PI/9,M_PI_4/2,-M_PI_4/3);
    std::cout << "wMc_true " << std::endl << wMc_true << std::endl;
    std::cout << "oMe_true " << std::endl << oMe_true << std::endl;
    vpHomogeneousMatrix base2gripper(1,-1,1,-M_PI_4/2,M_PI_4/5,-M_PI_4/7);
    eMw.push_back(base2gripper);
    cMo.push_back(wMc_true.inverse()*base2gripper.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper1(0,-1,1,M_PI_4/3,M_PI_4/7,M_PI_4/2);
    eMw.push_back(base2gripper1);
    cMo.push_back(wMc_true.inverse()*base2gripper1.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper2(0,1.2,-3,M_PI_2/3,-M_PI_4/7,-M_PI_2/5);
    eMw.push_back(base2gripper2);
    cMo.push_back(wMc_true.inverse()*base2gripper2.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper3(-0.9,-1.2,-0.1,M_PI_2/9,M_PI_4/15,M_PI_2/25);
    eMw.push_back(base2gripper3);
    cMo.push_back(wMc_true.inverse()*base2gripper3.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper4(1.9,0.2,5.1,M_PI_2/1,-M_PI_4/4,-M_PI_2/3);
    eMw.push_back(base2gripper4);
    cMo.push_back(wMc_true.inverse()*base2gripper4.inverse()*oMe_true.inverse());

    vpHomogeneousMatrix base2gripper5(2,1.12,0.1,-M_PI_2/4,-M_PI_4/5,-M_PI_2/25);
    eMw.push_back(base2gripper5);
    cMo.push_back(wMc_true.inverse()*base2gripper5.inverse()*oMe_true.inverse());

    if(vpHandEyeCalibration::calibrate(cMo,eMw,wMc)==0){
        std::cout << " Result " << std::endl << wMc << std::endl;
    }
    else{
        std::cout << " Fail " << std::endl;
    }

}




