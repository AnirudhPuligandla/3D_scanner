#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <iostream>
#include "cloudregister.h"
#include <QWidget>
#include "centralwidget.h"
#include "screensetup.h"

using namespace std;

//typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointT;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    /**--------Function to setup the GUI-------->*/
    void setupUi(MainWindow &w);


private:
    Ui::MainWindow *ui;

    // Objects to various classes
    CloudRegister cloudReg;
    CentralWidget widget;
    ScreenSetup getSize;
    QSize size;

    //-------------Data variables------------>
    std::string path;
    //dimensions for screen size
    int x_,y_;
    //central widget
    QWidget *centralWidget_;

    //pointer to read point cloud
    pcl::PointCloud<PointT>::Ptr cloud;

};

#endif // MAINWINDOW_H
