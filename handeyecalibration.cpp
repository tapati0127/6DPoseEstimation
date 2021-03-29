#include "handeyecalibration.h"

HandEyeCalibration::HandEyeCalibration()
{   
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
                 NULL, NULL, &align_to_color);

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
       vpDetectorAprilTag detector(tagFamily);

       detector.setAprilTagQuadDecimate(quad_decimate);
       detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
       detector.setAprilTagNbThreads(nThreads);
       detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
       detector.setZAlignedWithCameraAxis(align_frame);

       while (true) {
         g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                   NULL, NULL, &align_to_color);
         vpImageConvert::convert(I_color, I);

         I_color2 = I_color;
         vpImageConvert::convert(I_color, I);
         vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
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
         std::vector<vpHomogeneousMatrix> cMo_vec;
         detector.detect(I, tagSize, cam, cMo_vec);

         std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
         std::vector<int> tags_id = detector.getTagsId();
         std::map<int, double> tags_size;
         tags_size[-1] = tagSize; // Default tag size
         std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);
         for (size_t i = 0; i < tags_corners.size(); i++) {
           vpHomogeneousMatrix cMo;
           double confidence_index;
           if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo, &confidence_index)) {
             QImage tmp;
             tmp = Convert::vispToQImage(I_color2);
             Q_EMIT framesReady(tmp,tmp);
               if (confidence_index > 0.5) {
                 cv::Mat test;
              Convert::ViSP2Mat(cMo,test);
              std::cout << "confidence_index" << confidence_index<< std::endl << test << std::endl;
                 //vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::none, 3);
             }
             else if (confidence_index > 0.25) {
               //vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::orange, 3);
                 std::cout << "confidence_index" << confidence_index << std::endl;
             }
             else {
               //vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::red, 3);
                 std::cout << "confidence_index" << confidence_index << std::endl;
             }
             //std::stringstream ss;
             //ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
             //vpDisplay::displayText(I_color2, 35 + i*15, 20, ss.str(), vpColor::red);
           }
         }
       }
     } catch (const vpException &e) {
       std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
     }
}




