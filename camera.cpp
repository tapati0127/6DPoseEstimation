#include "camera.h"

void Camera::ConnectCamera(){
    tm->stop();
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0){
        std::cout << "Cannot connect with camera, reconnect after 2s..." << std::endl;

        tm->setInterval(2000);
        tm->start();
    }
    else{
        // Enable depth stream with given resolution. Pixel will have a bit depth of 16 bit
        cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, fps);

        // Enable RGB stream as frames with 3 channel of 8 bit
        cfg.enable_stream(RS2_STREAM_COLOR, rgb_width, rgb_height, RS2_FORMAT_RGB8, fps);


        auto pro = pipe.start();
        rs2::device dev = pro.get_device();
        auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
        std::ifstream file(cameraFile);
        std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        advanced_mode_dev.load_json(str);
        camera_running = true;
        Q_EMIT connected();
    }
}

void Camera::maxRangeChanged(int value)
{
    maxRange=value/1000.0;
}
Camera::Camera(int rgb_width, int rgb_height, int depth_width, int depth_height, int fps, std::string cameraFile)
{
    this->rgb_width=rgb_width;
    this->rgb_height=rgb_height;
    this->depth_width=depth_width;
    this->depth_height=depth_height;
    this->fps=fps;
    this->cameraFile=cameraFile;
    tm = new QTimer(this);
    tm->connect(tm,&QTimer::timeout,this,&Camera::ConnectCamera);
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
    qRegisterMetaType<cv::Mat>();
    ConnectCamera();



}

void Camera::run()
{
    while(!camera_running){
        if(thread_stop) return;
    }
    pcl.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    rs2::colorizer c;
    while(camera_running)
    {
        // Wait for frames and get them as soon as they are ready
        frames = pipe.wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        frames = align_to_color.process(frames);

        // Let's get our depth frame
        rs2::depth_frame depth = frames.get_depth_frame();

        //auto colorized_depth = c.colorize(depth);



        rs2::decimation_filter dec_filter(2);  // Decimation - reduces depth frame density
        rs2::sequence_id_filter seq_filter(2);
        rs2::threshold_filter thres_filter(0.15,maxRange);
        rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
        rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noi
        rs2::disparity_transform depth_to_disparity(true);
        rs2::disparity_transform disparity_to_depth(false);
        //depth = dec_filter.process(depth);
        depth = seq_filter.process(depth);
        depth = thres_filter.process(depth);
        depth = spat_filter.process(depth);
        depth = temp_filter.process(depth);
        //depth = depth_to_disparity.process(depth);


        auto inrist = rs2::video_stream_profile(depth.get_profile()).get_intrinsics();
        // And our rgb frame
        rs2::frame rgb = frames.get_color_frame();
        rs2::video_frame colorized_depth = rgb.as<rs2::video_frame>();
        int stride =colorized_depth.get_stride_in_bytes();
        uint8_t* ptr = (uint8_t*)colorized_depth.get_data();
        //uint8_t* ptr = (uint8_t*)rgb.get_data();
        //int stride = rgb.as<rs2::video_frame>().get_stride_in_bytes();

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
                    P[0] = -(float)(x-inrist.ppx)*P[2]/inrist.fx;
                    P[1] = (float)(y-inrist.ppy)*P[2]/inrist.fy;
                    P[2] = -P[2];
                    memcpy(pc.ptr<float>(index),P,sizeof(P));
                    pcl->points.at(index).x =P[0];
                    pcl->points.at(index).y =P[1];
                    pcl->points.at(index).z =P[2];
                    pcl->points.at(index).r =int(ptr[y * stride + (3*x)    ]);
                    pcl->points.at(index).g =int(ptr[y * stride + (3*x) + 1]);
                    pcl->points.at(index).b =int(ptr[y * stride + (3*x) + 2]);

                    index++;
                }
            }
        }
        pc.resize(index);
        cloud = pc;
        pcl->points.resize(index);

        emit pclReady(pcl);
        emit pointCloudReady(cloud);
        if(thread_stop) return;
    }
}

