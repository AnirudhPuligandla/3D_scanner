#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace std;

 // typedef pcl::PointXYZ PointT;
 typedef pcl::PointXYZRGBA PointT;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    // set background to black (R = 0, G = 0, B = 0)
    viewer.setBackgroundColor (0, 0, 0);
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    // you can add something here, ex:  add text in viewer
}


int main (int argc, char *argv[])
{
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered1 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);



    // Load .pcd file from argv[1]  3,6
    // /home/abdelrahman/3D_Scanner/Trials_and_Drafts/pcd_visualization_shubham/pcl-read-vis/cloud1.pcd
    int ret = pcl::io::loadPCDFile ("../PC-Gopi-original/cloud6.pcd", *cloud);

    if (ret < 0) {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return -1;
    }

    // Create the passthrough filtering object
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0,1.8);
      //pass.setFilterFieldName ("y");
      //pass.setFilterLimits (-1.0,4.0);

      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered1);

     //Create the statistical-outlier-removal filtering object
     pcl::StatisticalOutlierRemoval<PointT> sor;
     sor.setInputCloud (cloud_filtered1);
     sor.setMeanK (50);
     sor.setStddevMulThresh (5.0);
     sor.filter (*cloud_filtered2);


    // writing the results to o/p pcd file
      pcl::io::savePCDFileASCII ("../PC-Gopi-filtered/fcloud6.pcd", *cloud_filtered2);


    pcl::visualization::CloudViewer viewer("Cloud Viewer"); // //// viewer

    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud_filtered2);

    // use the following functions to get access to the underlying more advanced/powerful
    // PCLVisualizer

    // This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ()) {
        // you can also do cool processing here
        // FIXME: Note that this is running in a separate thread from viewerPsycho
        // and you should guard against race conditions yourself...
    }

    cout << "The code is running"<< endl;
    return 0;
}
