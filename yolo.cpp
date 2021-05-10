#include "yolo.h"

Yolo::Yolo(string path)
{
    // Give the configuration and weight files for the model
    String modelConfiguration = path + "/yolov4.cfg";
    String modelWeights = path + "/yolov4.weights";
    net = readNetFromDarknet(modelConfiguration, modelWeights);

    //net.setPreferableBackend(DNN_TARGET_OPENCL);
    net.setPreferableTarget(DNN_TARGET_OPENCL);

    string classesFile = "/classes.txt";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

}

void Yolo::postprocess(Mat &frame, const vector<Mat> &outs,vector<Yolo::yoloResult> &results)
{
    results.clear();
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    yoloResult yolo_result;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
//        drawPred(classIds[idx], confidences[idx], box.x, box.y,
//                 box.x + box.width, box.y + box.height, frame);
        yolo_result.classID = classIds[idx];
        yolo_result.x = box.x;
        yolo_result.y = box.y;
        yolo_result.pixW = box.width;
        yolo_result.pixH = box.height;
        cout << yolo_result.classID << ";" << yolo_result.x << ";" << yolo_result.y << ";" << yolo_result.pixW << ";" << yolo_result.pixH << endl;
        results.push_back(yolo_result);
    }
}

vector<String> Yolo::getOutputsNames(const Net &net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

void Yolo::computeYolo( Mat &frame, vector<Yolo::yoloResult> &results)
{
    //imshow("test",frame);
    Mat blob;
    // Create a 4D blob from a frame.
    blobFromImage(frame, blob, 1/255.0, cv::Size(inpWidth, inpHeight), Scalar(0,0,0), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward(outs, getOutputsNames(net));

    // Remove the bounding boxes with low confidence
    postprocess(frame, outs,results);

}
