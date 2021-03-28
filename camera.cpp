#include "camera.h"

Camera::Camera(int rgb_width, int rgb_height, int depth_width, int depth_height, int fps)
{
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0)
    throw std::runtime_error("No device detected. Is it plugged in?");
    // Enable depth stream with given resolution. Pixel will have a bit depth of 16 bit
    cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, fps);

    // Enable RGB stream as frames with 3 channel of 8 bit
    cfg.enable_stream(RS2_STREAM_COLOR, rgb_width, rgb_height, RS2_FORMAT_RGB8, fps);

    // Start our pipeline
    pipe.start(cfg);
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
    qRegisterMetaType<cv::Mat>();
}

void Camera::run()
{
    pcl.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    while(camera_running)
    {
        // Wait for frames and get them as soon as they are ready
        frames = pipe.wait_for_frames();

        // Let's get our depth frame
        rs2::depth_frame depth = frames.get_depth_frame();
        auto inrist = rs2::video_stream_profile(depth.get_profile()).get_intrinsics();
        // And our rgb frame
        rs2::frame rgb = frames.get_color_frame();
        uint8_t* ptr = (uint8_t*)rgb.get_data();
        int stride = rgb.as<rs2::video_frame>().get_stride_in_bytes();
        // Let's convert them to QImage
        auto q_rgb = realsenseFrameToQImage(rgb);
        auto q_depth = realsenseFrameToQImage(depth);

        // And finally we'll emit our signal
        emit framesReady(q_rgb, q_depth);


        cv::Mat pc = cv::Mat(depth.get_height()*depth.get_width(), 3, CV_32FC1);
        // Setup the cloud pointer
        // The number of points in the cloud
        pcl->points.resize(depth.get_height()*depth.get_width());

        size_t index = 0;
        for(int y=0; y<depth.get_height(); ++y)
        {
            for(int x=0; x<depth.get_width(); ++x)
            {
                float P[3];
                P[2] = depth.get_distance(x,y);
                //rs2_deproject_pixel_to_point(planarPoint3d, &inrist, pixel, pixel_distance_in_meters);
                if(P[2]>0.00001&&P[2]<0.6){
                    P[0] = (float)(x-inrist.ppx)*P[2]/inrist.fx;
                    P[1] = (float)(y-inrist.ppy)*P[2]/inrist.fy;
                    memcpy(pc.ptr<float>(index),P,sizeof(P));
                    pcl->points.at(index).x =P[0];
                    pcl->points.at(index).y =P[1];
                    pcl->points.at(index).z =P[2];
                    pcl->points.at(index).r =255;
                    pcl->points.at(index).g =0;
                    pcl->points.at(index).b =0;
                    index++;
                }
            }
        }
        pc.resize(index);
        cloud = pc;
        pcl->points.resize(index);

        emit pclReady(pcl);
        emit pointCloudReady(cloud);
    }
}

QImage realsenseFrameToQImage(const rs2::frame &f)
{
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

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
