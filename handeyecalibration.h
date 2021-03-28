#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H
#include <QThread>
#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/vision/vpPose.h>
#include <librealsense2/rs.hpp> 


class HandEyeCalibration: public QThread
{
    Q_OBJECT
public:
    HandEyeCalibration();
    //static int startHandEyeCalibration();
    int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou);
    int ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou);
    // Member function that handles thread iteration
    void run();
    // If called it will stop the thread
    void stop();
private:

};

#endif // HANDEYECALIBRATION_H
