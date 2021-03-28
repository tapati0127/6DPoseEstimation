#ifndef PCLVIEWER_H
#define PCLVIEWER_H
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include "QVTKWidget.h"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class pclViewer
{
public:
    pclViewer();
    pclViewer(QVTKWidget* vtk);
    ~pclViewer(){};
    void display();
    void displayPCL(PointCloudT::Ptr pc);
    bool isPclAdded = false;
protected:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  PointCloudT::Ptr cloud;

  unsigned int red;
  unsigned int green;
  unsigned int blue;
private:
  QVTKWidget* vtk;
};

#endif // PCLVIEWER_H
