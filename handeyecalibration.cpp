#include "handeyecalibration.h"

HandEyeCalibration::HandEyeCalibration()
{
    
}

void HandEyeCalibration::run()
{
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.053;
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

  #ifdef VISP_HAVE_GDI
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
  
      std::vector<double> time_vec;
      for (;;) {
        double t = vpTime::measureTimeMs();
  
        g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
                  NULL, NULL, &align_to_color);
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
            if (confidence_index > 0.75) {
              vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::none, 3);
            }
            else if (confidence_index > 0.25) {
              vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::orange, 3);
            }
            else {
              vpDisplay::displayFrame(I_color2, cMo, cam, tagSize/2, vpColor::red, 3);
            }
            std::stringstream ss;
            ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
            vpDisplay::displayText(I_color2, 35 + i*15, 20, ss.str(), vpColor::red);
          }
        }
  
        vpDisplay::displayText(I_color, 20, 20, "Pose from homography + VVS", vpColor::red);
        vpDisplay::displayText(I_color2, 20, 20, "Pose from RGBD fusion", vpColor::red);
        vpDisplay::displayText(I_color, 35, 20, "Click to quit.", vpColor::red);
        t = vpTime::measureTimeMs() - t;
        time_vec.push_back(t);
  
        std::stringstream ss;
        ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
        vpDisplay::displayText(I_color, 50, 20, ss.str(), vpColor::red);
  
        if (vpDisplay::getClick(I_color, false))
          break;
  
        vpDisplay::flush(I_color);
        vpDisplay::flush(I_color2);
        vpDisplay::flush(I_depth);
      }
  
      std::cout << "Benchmark loop processing time" << std::endl;
      std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
                << " ; " << vpMath::getMedian(time_vec) << " ms"
                << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;
  
      if (! display_off) {
        delete d1;
        delete d2;
        delete d3;
      }
  
    } catch (const vpException &e) {
      std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    }
}

/**
 * @brief copy OpenCV Mat to ViSP vpHomogeneousMatrix
 */
int HandEyeCalibration::Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou)
{
  int ret = 0;

  //if (mat_in.type() != CV_32FC1 || mat_in.type() != CV_64FC1)
  if (mat_in.type() != CV_64FC1)
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
      visp_ou[i][j] = mat_in.ptr<double>(i)[j];  // new memory is created and data is copied in this line
    }
  }

  return ret;
}

/**
 * @brief copy ViSP vpHomogeneousMatrix to OpenCV Mat
 */
int HandEyeCalibration::ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou)
{
  int ret = 0;

  mat_ou = cv::Mat::zeros(visp_in.getRows(),visp_in.getCols(),CV_64FC1);

  for (int i=0; i<mat_ou.rows; i++)
  {
    for (int j=0; j<mat_ou.cols; j++)
    {
      mat_ou.ptr<double>(i)[j] = visp_in[i][j];  // new memory is created and data is copied in this line
    }
  }

  return ret;
}

