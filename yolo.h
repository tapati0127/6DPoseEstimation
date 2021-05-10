#ifndef YOLO_H
#define YOLO_H
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace dnn;
using namespace std;






class Yolo
{
public:
    Yolo(string path);


    // Draw the predicted bounding box
//    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

    // Get the names of the output layers
    vector<String> getOutputsNames(const Net& net);
    struct yoloResult
    {
        int classID; int x; int y; int pixW; int pixH;
    };
    void computeYolo( Mat& frame,vector<struct yoloResult> &results);
    // Remove the bounding boxes with low confidence using non-maxima suppression
    void postprocess(Mat& frame, const vector<Mat>& out,vector<Yolo::yoloResult> &results);

private:
    vector<string> classes;
    Net net;
    // Initialize the parameters
    float confThreshold = 0.5; // Confidence threshold
    float nmsThreshold = 0.4;  // Non-maximum suppression threshold
    int inpWidth = 416;  // Width of network's input image
    int inpHeight = 416; // Height of network's input image
};

#endif // YOLO_H
