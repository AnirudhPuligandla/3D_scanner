#include "cloudDisplay.h"

CloudDisplay::CloudDisplay(QWidget *parent) : QVTKWidget(parent)
{
    //displayWidget_ = new QVTKWidget(this);
    this->SetRenderWindow(m_visualizer.getRenderWindow());
}

QVTKWidget* CloudDisplay::getVTKWidget()
{
    return displayWidget_;
}
