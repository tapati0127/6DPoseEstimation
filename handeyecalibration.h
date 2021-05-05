#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H
#include <QThread>
#include <QImage>
#include <QMetaType>
#include <visp3/core/vpConfig.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/vision/vpPose.h>
#include <librealsense2/rs.hpp> 
#include "convert.h"
#include "motoudp.h"
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
Q_DECLARE_METATYPE(Eigen::Affine3f)
using namespace Convert;
class HandEyeCalibration: public QThread
{
    Q_OBJECT
public:
    HandEyeCalibration(const QString &ip,std::string cameraFile);
    // Member function that handles thread iteration
    void run();
    // If called it will stop the thread
    void stop();
    void startTrigger(){trigger=true;}
    void startSendPosition(){sendPosition=true;}
    void receivedPosition(int32_t* pos);
    void caculatePose();
    void test();
    Matx44d calib_pose;
signals:
    // A signal sent by our class to notify that there are frames that need to be processed
    void framesReady(QImage frameRGB, QImage frameDepth);
    void finishCalibrate(Eigen::Affine3f aswer);
private:
    bool trigger = false;
    bool isReceivePositionFromRobot = false;
    MotoUDP* motoudp;
    std::vector<cv::Mat> R_base2gripper;
    std::vector<vpHomogeneousMatrix> cMo;
    std::vector<cv::Mat> t_base2gripper;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    std::vector<vpHomogeneousMatrix> eMw;
    cv::Mat R_cam2base,t_cam2base;
    vpHomogeneousMatrix wMc,oMe;
    bool isSimulation = true;
    bool thread_stop=false;
     vpRealSense2 g;
     std::string cameraFile;
    //cv::Mat R_base2cam_,t_base2cam_;

    //
    bool sendPosition = false;
    //


};

#endif // HANDEYECALIBRATION_H
