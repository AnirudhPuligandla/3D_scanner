#include "myvtkwidget.h"
#include "kinect2_grabber.h"
#include <string.h>

MyVTKWidget::MyVTKWidget(QWidget *parent) : QWidget(parent)
{
    //initialize widgets and layouts
    centralWidget_ = new QWidget(this);
    sideContainer = new QWidget(this);
    buttonContainer = new QWidget(this);
    mainLayout_ = new QHBoxLayout;
    sideLayout_ = new QVBoxLayout;
    buttonLayout = new QVBoxLayout;

    //Initialize Progress Bar
    progressBar = new QProgressBar(this);

    //Intialize VTK Widget
    //startButton_ = new QPushButton(this);
    displayWidget_ = new QVTKWidget;
    vtkObject::GlobalWarningDisplayOff();

    //Initialize startButton
    readButton_ = new QPushButton("Read", this);
    scanButton = new QPushButton("Scan", this);

    //Initialize the cloud list widget and connect it to the slot
    cloudList = new QListWidget(this);
    QObject::connect(cloudList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(on_cloudListItem_clicked(QListWidgetItem*)));

    //Connect the pushButton pressed signal to its slot
    QObject::connect(readButton_, SIGNAL(pressed()), this, SLOT(on_readButton_pressed()));
    QObject::connect(scanButton, SIGNAL(pressed()), this, SLOT(on_scanButton_pressed()));

    //initialize point cloud
    cloud.reset(new pcl::PointCloud<PointT>);

    /*qDebug() << "Calling get count";
    count = fileGrab.getCount();

    for(int i=0; i < count; i++)
    {

        cloudVector.push_back(cloud);
    }*/

    //initialize PCL m_visualizer
    m_visualizer.reset(new pcl::visualization::PCLVisualizer ("3D m_visualizer", false));
    m_visualizer->setBackgroundColor(0.1,0.1,0.1);

    //renderWindow = m_visualizer->getRenderWindow();
    displayWidget_->SetRenderWindow(m_visualizer->getRenderWindow());
    m_visualizer->setupInteractor(displayWidget_->GetInteractor(), displayWidget_->GetRenderWindow());
}

QWidget* MyVTKWidget::getCentralWidget()
{
    //get the screen resolution
    ScreenSetup screenSetup;
    QSize size = screenSetup.size();
    int x = size.width();
    int y = size.height();

    //set the dimensions and positions for widgets
    //displayWidget_->setFixedSize(x,(3*(y/4)));
    //displayWidget_->setGeometry((x/4),0,x,y);
    //startButton_->setFixedSize(x,(y/4));
    //sideContainer->setGeometry(0,0, (x/4), y);
    sideContainer->setFixedWidth((x/4));

    //setup central widget
    mainLayout_->setSpacing(5);
    mainLayout_->setContentsMargins(5,0,5,0);

    buttonLayout->setSpacing(5);
    sideLayout_->setSpacing(5);
    sideLayout_->setContentsMargins(5,5,5,5);

    buttonLayout->addWidget(readButton_);
    buttonLayout->addWidget(scanButton);
    buttonContainer->setLayout(buttonLayout);

    sideLayout_->addWidget(buttonContainer);
    sideLayout_->addWidget(cloudList);
    sideLayout_->addWidget(progressBar);
    sideContainer->setLayout(sideLayout_);

    mainLayout_->addWidget(sideContainer);
    mainLayout_->addWidget(displayWidget_);
    centralWidget_->setLayout(mainLayout_);

    return centralWidget_;
}

void MyVTKWidget::on_readButton_pressed()
{
    //Read and display the point cloud
    qDebug() << "read pressed";

    fileGrab.initializeFileList();
    pcdNames = fileGrab.getNames();

    //access the names
    vector<QString>::iterator it = pcdNames.begin();
    while(it != pcdNames.end())
    {
        cloudList->addItem(*it);
        //qDebug () << *it <<" ";
        it++;
    }

    cloudVector = fileGrab.getPointClouds();

    //Display the first point cloud in the list
    m_visualizer->addPointCloud(cloudVector[0], "ccc");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ccc");
    m_visualizer->addCoordinateSystem();
    displayWidget_->update();
    /*
    pcl::io::loadPCDFile(path, *cloud);
    qDebug() << cloud->points.size();
    m_visualizer->addPointCloud(cloud, "ccc");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ccc");
    m_visualizer->addCoordinateSystem();
    displayWidget_->update();*/
}

void MyVTKWidget::on_cloudListItem_clicked(QListWidgetItem *item)
{
    for(int i = 1; i <= cloudVector.size(); i++)
    {
        if(cloudList->item(i) == item)
        {
            m_visualizer->removePointCloud("ccc");
            m_visualizer->addPointCloud(cloudVector[i-1], "ccc");
            //m_visualizer->spinOnce();
            displayWidget_->update();
            qDebug() << "point cloud updated" << i;
        }
    }
}

void MyVTKWidget::on_scanButton_pressed()
{
    qDebug () << "hello";
    unsigned int fileSaved =0;
    pcl::PointCloud<PointT>::ConstPtr cloud;
    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointT>::ConstPtr& )> pointcloud_function =
            [&cloud, &mutex]( const pcl::PointCloud<PointT>::ConstPtr& ptr ){
        boost::mutex::scoped_lock lock( mutex );
        cloud = ptr;
    };

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( pointcloud_function );

    // Keyboard Callback Function and saving PCD file - (NEW function added)
    //int fileSaved = 0;
    boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard_function =
            [&cloud, &mutex, &fileSaved]( const pcl::visualization::KeyboardEvent& event ){
        // Save Point Cloud to PCD File when Pressed Space Key
        if( event.getKeyCode() == VK_SPACE && event.keyDown() ){
            boost::mutex::scoped_try_lock lock( mutex );
            if(lock.owns_lock()){
                //pcl::io::savePCDFile("cloud1", *cloud, false);
                stringstream stream;
                stream << "InputCloud" << fileSaved << ".pcd";
                string filename = stream.str();
                if(pcl::io::savePCDFile(filename, *cloud, false )==0)
                {
                    fileSaved++;
                    cout << "Saved" << filename << "." << endl;
                }
                else PCL_ERROR("Problem saving %s.\n",filename.c_str());
            }
        }
    };

    // Register Callback Function
    m_visualizer->registerKeyboardCallback( keyboard_function );

    // Start Grabber
    grabber->start();

    qDebug () << "started";

    for(int i=0; i <= 10000; i++){
        // Update m_visualizer
        //m_visualizer->spinOnce();

        qDebug () << "boom";

        boost::mutex::scoped_try_lock lock( mutex );
        if( cloud && lock.owns_lock() ){
            if( cloud->size() != 0 ){
                /* Processing Point Cloud */

                // Update Point Cloud
                if( !m_visualizer->updatePointCloud( cloud, "cloud" ) ){
                    m_visualizer->addPointCloud( cloud, "cloud" );
                    m_visualizer->resetCameraViewpoint( "cloud" );
                    displayWidget_->update();
                }
            }
        }
    }

    // Stop Grabber
    grabber->stop();
}
