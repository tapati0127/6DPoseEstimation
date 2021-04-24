#ifndef PPF_H
#define PPF_H
#include <QThread>
#include "/home/tapati/OpenCV/opencv_contrib/modules/surface_matching/include/opencv2/surface_matching/ppf_helpers.hpp"
#include "/home/tapati/OpenCV/opencv_contrib/modules/surface_matching/include/opencv2/surface_matching/ppf_match_3d.hpp"
#include "/home/tapati/OpenCV/opencv_contrib/modules/surface_matching/include/opencv2/surface_matching.hpp"
using namespace  cv;
using namespace  std;
using namespace  ppf_match_3d;
class PPF:public QThread
{
    Q_OBJECT
public:
    PPF(std::string modelPath);
    void run();
    bool caculatePPF(const Mat &pc,Pose3DPtr &result);
    bool caculatePPF(const Mat &pc,Pose3DPtr &result,Mat &pc_result);
    bool isComplete = false;
signals:
    void complete();
private:

    std::string modelPath;
    PPF3DDetector* detector;


};

#endif // PPF_H
