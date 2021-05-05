#include "ppf.h"
#include <iostream>
#include <fstream>

const char *DETECTOR_FILENAME = "detector.xml";

std::string dirnameOf(const std::string& fname)
{
     size_t pos = fname.find_last_of("\\/");
     return (std::string::npos == pos)
         ? ""
         : fname.substr(0, pos);
}

PPF::PPF(std::string modelPath)
{
    this->modelPath = modelPath;


}

void PPF::run()
{
    int64 tick1, tick2;
    Mat model = loadPLYSimple(modelPath.c_str(), 1);
    detector = new PPF3DDetector(0.025, 0.05);

    std::string dir = dirnameOf(modelPath);
    if(!detector->readFile(dir)){
        cout << "Training..." << endl;
        tick1 = cv::getTickCount();
        detector->trainModel(model);
        tick2 = cv::getTickCount();
        cout << "Training complete in "
             << (double)(tick2 - tick1) / cv::getTickFrequency()
             << " sec" << endl;
        // Serialize the model
        cout << "Serializing..." << endl;
        tick1 = cv::getTickCount();
        detector->writeFile(dir);
        tick2 = cv::getTickCount();

        cout << "Serialization complete in "
             << (double)(tick2 - tick1) / cv::getTickFrequency()
             << " sec" << endl;
    }
    else{
        cout << "Found detector file: Skipping training phase" << endl;
    }
      isComplete = true;
      Q_EMIT complete();
}

bool PPF::caculatePPF(const Mat &pc, Pose3DPtr &result)
{
    Mat pc_normal;
    int64 tick1, tick2;
    tick1 = cv::getTickCount();
    ppf_match_3d::computeNormalsPC3d(pc,pc_normal,10,true,Vec3f(0,0,0));
    tick2 = cv::getTickCount();
    cout << endl << "Normal Estimation Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;
    tick1 = cv::getTickCount();
    vector<Pose3DPtr> results;
    detector->match(pc_normal, results, 1.0/5, 0.05);
    tick2 = cv::getTickCount();
    cout << endl << "PPF Elapsed Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;

    //check results size from match call above
    size_t results_size = results.size();
    cout << "Number of matching poses: " << results_size;
    if (results_size == 0) {
        cout << endl << "No matching poses found. Exiting." << endl;
        return false;
    }


    // Get only first N results - but adjust to results size if num of results are less than that specified by N
        size_t N = 2;
        if (results_size < N) {
            cout << endl << "Reducing matching poses to be reported (as specified in code): "
                 << N << " to the number of matches found: " << results_size << endl;
            N = results_size;
        }
        vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);

        // Create an instance of ICP
        ICP icp(100, 0.005f, 2.5f, 8);
        int64 t1 = cv::getTickCount();

        // Register for all selected poses
        //cout << endl << "Performing ICP on " << N << " poses..." << endl;
        icp.registerModelToScene(detector->model, pc_normal, resultsSub);
        int64 t2 = cv::getTickCount();

        cout << endl << "ICP Elapsed Time " <<
             (t2-t1)/cv::getTickFrequency() << " sec" << endl;
//        cout << endl << "There are " << resultsSub.size() << " poses" << endl;
//        for (int i=0;i<resultsSub.size();i++) {
//            Mat pct = transformPCPose(detector->model, resultsSub.at(i)->pose);
//        }
        cout << "pose " << resultsSub.at(0)->pose << endl;
        cout << "numVotes " << resultsSub.at(0)->numVotes << endl;
        cout << "residual " << resultsSub.at(0)->residual << endl;
        result = resultsSub.at(0);
        return true;
}

bool PPF::caculatePPF(const Mat &pc, Pose3DPtr &result, Mat &pc_result)
{
    Mat pc_normal;
    int64 tick1, tick2;
    tick1 = cv::getTickCount();
    ppf_match_3d::computeNormalsPC3d(pc,pc_normal,10,true,Vec3f(0,0,0));
    tick2 = cv::getTickCount();
    cout << endl << "Normal Estimation Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;
    tick1 = cv::getTickCount();
    vector<Pose3DPtr> results;
    detector->match(pc_normal, results, 1.0/5, 0.05);
    tick2 = cv::getTickCount();
    cout << endl << "PPF Elapsed Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;

    //check results size from match call above
    size_t results_size = results.size();
    cout << "Number of matching poses: " << results_size;
    if (results_size == 0) {
        cout << endl << "No matching poses found. Exiting." << endl;
        return false;
    }


    // Get only first N results - but adjust to results size if num of results are less than that specified by N
        size_t N = 2;
        if (results_size < N) {
            cout << endl << "Reducing matching poses to be reported (as specified in code): "
                 << N << " to the number of matches found: " << results_size << endl;
            N = results_size;
        }
        vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);

        // Create an instance of ICP
        ICP icp(100, 0.005f, 2.5f, 8);
        int64 t1 = cv::getTickCount();

        // Register for all selected poses
        cout << endl << "Performing ICP on " << N << " poses..." << endl;
        icp.registerModelToScene(detector->model, pc_normal, resultsSub);
        int64 t2 = cv::getTickCount();

        cout << endl << "ICP Elapsed Time " <<
             (t2-t1)/cv::getTickFrequency() << " sec" << endl;
        //cout << endl << "There are " << resultsSub.size() << " poses" << endl;
        pc_result = transformPCPose(detector->model, resultsSub.at(0)->pose);
        cout << "pose " << resultsSub.at(0)->pose << endl;
        cout << "numVotes " << resultsSub.at(0)->numVotes << endl;
        cout << "residual " << resultsSub.at(0)->residual << endl;
        result = resultsSub.at(0);
        return true;
}
