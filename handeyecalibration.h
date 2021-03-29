#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H
#include <QThread>
#include <QImage>
#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/vision/vpPose.h>
#include <librealsense2/rs.hpp> 
#include "convert.h"

class HandEyeCalibration: public QThread
{
    Q_OBJECT
public:
    HandEyeCalibration();
    //static int startHandEyeCalibration();

    // Member function that handles thread iteration
    void run();
    // If called it will stop the thread
    void stop();
signals:
    // A signal sent by our class to notify that there are frames that need to be processed
    void framesReady(QImage frameRGB, QImage frameDepth);
private:

};

#endif // HANDEYECALIBRATION_H
