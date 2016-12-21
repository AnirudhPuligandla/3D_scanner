#include "filegrabber.h"

void FileGrabber::initializeFileList()
{
    //Initialize the path
    path = "C:\\Users\\pvani_000\\Downloads\\data";

    //Count the number of files in the directory
    QDir dir(path);
    dir.setFilter( QDir::AllEntries | QDir::NoDotAndDotDot );
    count = dir.count();

    //initialize point cloud
    cloud.reset(new pcl::PointCloud<PointT>);

    //Initialize the point cloud Vector
    for(int i=0; i < count; i++)
    {

        cloudVector.push_back(cloud);
    }

    //qDebug() << "Fuck You";

    //Iterate over the directory getting files with .pcd extension
    QDirIterator it(path, QStringList() << "*.pcd", QDir::Files);

    int i = 0;

    while (it.hasNext())
    {
        //Save the paths to the files
        pcdPaths.push_back(it.next().toStdString());

        //Save the file names
        pcdNames.push_back(it.fileName());

        //read the point clouds and add to the vector
        pcl::io::loadPCDFile(it.filePath().toStdString(), *cloudVector[i]);
        i++;

        qDebug() << it.filePath();
    }
}

vector<string> FileGrabber::getPaths()
{
    return pcdPaths;
}

vector<QString> FileGrabber::getNames()
{
    return pcdNames;
}

int FileGrabber::getCount()
{
    return count;
}

vector<pcl::PointCloud<PointT>::Ptr> FileGrabber::getPointClouds()
{
    return cloudVector;
}
