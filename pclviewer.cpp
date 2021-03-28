#include "pclviewer.h"

pclViewer::pclViewer()
{

    /*
    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize (200);

    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // Fill the cloud with some points
    for (auto& point: *cloud)
    {
      point.x = 1024 * rand () / (RAND_MAX + 1.0f);
      point.y = 1024 * rand () / (RAND_MAX + 1.0f);
      point.z = 1024 * rand () / (RAND_MAX + 1.0f);

      point.r = red;
      point.g = green;
      point.b = blue;
    }*/
}

pclViewer::pclViewer(QVTKWidget *vtk)
{
    this->vtk = vtk;
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    vtk->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setBackgroundColor(1,1,1);
    viewer->setupInteractor (vtk->GetInteractor (), vtk->GetRenderWindow());
    vtk->update ();
}

void pclViewer::display()
{
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    vtk->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setBackgroundColor(1,1,1);

    viewer->setupInteractor (vtk->GetInteractor (), vtk->GetRenderWindow());
    vtk->update ();
    viewer->addPointCloud (cloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud");
    viewer->resetCamera();
    vtk->update();
}

void pclViewer::displayPCL(PointCloudT::Ptr pc)
{
    if(!isPclAdded){
        viewer->addPointCloud (pc, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->resetCamera();
        vtk->update();
        isPclAdded=true;
    }
    else{
        viewer->updatePointCloud(pc,"cloud");
        //viewer->resetCamera();
        vtk->update();
    }

}