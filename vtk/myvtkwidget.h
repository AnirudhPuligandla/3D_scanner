#ifndef MYVTKWIDGET_H
#define MYVTKWIDGET_H

#include <QWidget>
#include <QVTKWidget.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <QDebug>
#include <QWidget>
#include <QListWidget>
#include <QListWidgetItem>
#include <iostream>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSize>
#include "screensetup.h"
#include <QDebug>
#include <QProgressBar>
#include "filegrabber.h"
#include <vector>
#include <QString>
#include <QDebug>
#include "kinect2_grabber.h"

using namespace std;

//typedef pcl::PointXYZRGBA PointT;
 typedef pcl::PointXYZRGBA PointT;

class MyVTKWidget : public QWidget
{
    Q_OBJECT

public:
    MyVTKWidget(QWidget *parent = 0);
    QWidget* getCentralWidget();

private:
    QPushButton *readButton_;
    QPushButton *scanButton;

    //-------------Data variables------------>
    vector<QString> pcdNames;
    //uint count;

    //vector containing all the point clouds
    vector<pcl::PointCloud<PointT>::Ptr> cloudVector;

    //point cloud
    pcl::PointCloud<PointT>::ConstPtr cloud;

    //PCL Viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;

    //widgets and layouts
    QVTKWidget *displayWidget_;
    QWidget *centralWidget_;
    QHBoxLayout *mainLayout_;
    QVBoxLayout *sideLayout_;
    QVBoxLayout *buttonLayout;
    QWidget *sideContainer;
    QWidget *buttonContainer;
    QProgressBar *progressBar;
    QListWidget *cloudList;

    //Objects
    FileGrabber fileGrab;


public slots:

    //startButton pressed slot
    void on_readButton_pressed();

    //Scan button pressed slot
    void on_scanButton_pressed();

    //point cloud list slot
    void on_cloudListItem_clicked(QListWidgetItem* item);
};

#endif // MYVTKWIDGET_H
