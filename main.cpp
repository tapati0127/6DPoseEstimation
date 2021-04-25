#include "mainwindow.h"
#include <QApplication>
//#include "convert.h"
//#include "opencv2/core.hpp"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
//   cv::Vec3f a(-179.9995*M_PI/180,20.000601*M_PI/180,-30.0009*M_PI/180);
//   std::cout << eulerAnglesToRotationMatrix(a) << std::endl;


}
