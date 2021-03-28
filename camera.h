#ifndef CAMERA_H
#define CAMERA_H
// Import QT libs, one for threads and one for images
#include <QThread>
#include <QImage>
#include <QMetaType>
// Import librealsense header
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core.hpp"
// Let's define our camera as a thread, it will be constantly running and sending frames to
// our main window

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)
Q_DECLARE_METATYPE(cv::Mat)
class Camera : public QThread
{
    Q_OBJECT
public:
    // We need to instantiate a camera with both depth and rgb resolution (as well as fps)
    Camera(int rgb_width, int rgb_height, int depth_width, int depth_height, int fps);
    ~Camera() {
        pipe.stop();
    }

    // Member function that handles thread iteration
    void run();

    // If called it will stop the thread
    void stop() {
        pipe.stop();
        camera_running = false; }


private:
    // Realsense configuration structure, it will define streams that need to be opened
    rs2::config cfg;

    // Our pipeline, main object used by realsense to handle streams
    rs2::pipeline pipe;

    // Frames returned by our pipeline, they will be packed in this structure
    rs2::frameset frames;

    // A bool that defines if our thread is running
    bool camera_running = true;

    cv::Mat cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl;

signals:
    // A signal sent by our class to notify that there are frames that need to be processed
    void framesReady(QImage frameRGB, QImage frameDepth);
    void pclReady(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud);
    void pointCloudReady(cv::Mat pointcloud);
};
// A function that will convert realsense frames to QImage
QImage realsenseFrameToQImage(const rs2::frame& f);
#endif // CAMERA_H
