#ifndef CAMERA_H
#define CAMERA_H
// Import QT libs, one for threads and one for images
#include <QThread>
#include <QImage>
#include <QMetaType>
#include <QTimer>
// Import librealsense header
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core.hpp"
#include "convert.h"
#include "ppf.h"
// Let's define our camera as a thread, it will be constantly running and sending frames to
// our main window
Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)
Q_DECLARE_METATYPE(cv::Mat)
class Camera : public QThread
{
    Q_OBJECT
public:
    // We need to instantiate a camera with both depth and rgb resolution (as well as fps)
    Camera(int rgb_width, int rgb_height, int depth_width, int depth_height, int fps, std::string cameraFile);
    ~Camera() {std::cout << "Camera Class off " << std::endl;}

    // Member function that handles thread iteration
    void run();

    // If called it will stop the thread
    void stop() {
        thread_stop = true;
        this->wait(600);
         }
    void setMaxRange(const int&value){maxRange=value/1000.0;}
    bool camera_running = false;
    bool capture = false;
signals:
    void connected();
private:
    // Realsense configuration structure, it will define streams that need to be opened
    rs2::config cfg;

    // Our pipeline, main object used by realsense to handle streams
    rs2::pipeline pipe;

    // Frames returned by our pipeline, they will be packed in this structure
    rs2::frameset frames;

    // A bool that defines if our thread is running

    bool thread_stop = false;
    double maxRange = 0.6;
    cv::Mat cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl;

    int rgb_width; int rgb_height; int depth_width; int depth_height; int fps; std::string cameraFile;
    QTimer* tm;
    int captureCount = 0;
signals:
    // A signal sent by our class to notify that there are frames that need to be processed
    void framesReady(QImage frameRGB, QImage frameDepth);
    void pclReady(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud);
    void pointCloudReady(cv::Mat pointcloud);
private slots:
    void ConnectCamera();
public slots:
    void maxRangeChanged(int value);
};
#endif // CAMERA_H
