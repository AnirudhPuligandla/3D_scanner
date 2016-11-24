#ifndef CLOUDDISPLAY_H
#define CLOUDDISPLAY_H

#include <QWidget>
#include <QVTKWidget.h>
#include <pcl/visualization/cloud_viewer.h>

class CloudDisplay : public QVTKWidget
{

public:
    CloudDisplay();
    CloudDisplay(QWidget *parent);
    QVTKWidget* getVTKWidget();

private:
    pcl::visualization::PCLVisualizer m_visualizer;
    QVTKWidget *displayWidget_;
};

#endif // CLOUDDISPLAY_H
