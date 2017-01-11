#include "kinect2_grabber.h"

using namespace pcl;

Kinect2Grabber::Kinect2Grabber()
    : sensor( nullptr )
    , mapper( nullptr )
    , colorSource( nullptr )
    , colorReader( nullptr )
    , depthSource( nullptr )
    , depthReader( nullptr )
    , infraredSource( nullptr )
    , infraredReader( nullptr )
    , result( S_OK )
    , colorWidth( 1920 )
    , colorHeight( 1080 )
    , colorBuffer()
    , depthWidth( 512 )
    , depthHeight( 424 )
    , depthBuffer()
    , infraredWidth( 512 )
    , infraredHeight( 424 )
    , infraredBuffer()
    , running( false )
    , quit( false )
    , available( true )
    , signal_PointXYZ( nullptr )
    , signal_PointXYZI( nullptr )
    , signal_PointXYZRGB( nullptr )
    , signal_PointXYZRGBA( nullptr )
{
    // Create Sensor Instance
    result = GetDefaultKinectSensor( &sensor );
    if( FAILED( result ) ){
        throw std::exception( "Exception : GetDefaultKinectSensor()" );
    }

    // Open Sensor
    result = sensor->Open();
    if( FAILED( result ) ){
        throw std::exception( "Exception : IKinectSensor::Open()" );
    }

    // Retrieved Coordinate Mapper
    result = sensor->get_CoordinateMapper( &mapper );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IKinectSensor::get_CoordinateMapper()" );
    }

    // Retrieved Color Frame Source
    result = sensor->get_ColorFrameSource( &colorSource );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IKinectSensor::get_ColorFrameSource()" );
    }

    // Retrieved Depth Frame Source
    result = sensor->get_DepthFrameSource( &depthSource );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IKinectSensor::get_DepthFrameSource()" );
    }

    // Retrieved Infrared Frame Source
    result = sensor->get_InfraredFrameSource( &infraredSource );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IKinectSensor::get_InfraredFrameSource()" );
    }
    if( infraredSource == nullptr){
        available = false;
    }

    // Retrieved Color Frame Size
    IFrameDescription* colorDescription;
    result = colorSource->get_FrameDescription( &colorDescription );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IColorFrameSource::get_FrameDescription()" );
    }

    result = colorDescription->get_Width( &colorWidth ); // 1920
    if( FAILED( result ) ){
        throw std::exception( "Exception : IFrameDescription::get_Width()" );
    }

    result = colorDescription->get_Height( &colorHeight ); // 1080
    if( FAILED( result ) ){
        throw std::exception( "Exception : IFrameDescription::get_Height()" );
    }

    SafeRelease( colorDescription );

    // To Reserve Color Frame Buffer
    colorBuffer.resize( colorWidth * colorHeight );

    // Retrieved Depth Frame Size
    IFrameDescription* depthDescription;
    result = depthSource->get_FrameDescription( &depthDescription );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IDepthFrameSource::get_FrameDescription()" );
    }

    result = depthDescription->get_Width( &depthWidth ); // 512
    if( FAILED( result ) ){
        throw std::exception( "Exception : IFrameDescription::get_Width()" );
    }

    result = depthDescription->get_Height( &depthHeight ); // 424
    if( FAILED( result ) ){
        throw std::exception( "Exception : IFrameDescription::get_Height()" );
    }

    SafeRelease( depthDescription );

    // To Reserve Depth Frame Buffer
    depthBuffer.resize( depthWidth * depthHeight );

    // Retrieved Infrared Frame Size
    IFrameDescription* infraredDescription;
    result = infraredSource->get_FrameDescription( &infraredDescription );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IInfraredFrameSource::get_FrameDescription()" );
    }

    result = infraredDescription->get_Width( &infraredWidth ); // 512
    if( FAILED( result ) ){
        throw std::exception( "Exception : IFrameDescription::get_Width()" );
    }

    result = infraredDescription->get_Height( &infraredHeight ); // 424
    if( FAILED( result ) ){
        throw std::exception( "Exception : IFrameDescription::get_Height()" );
    }

    SafeRelease( infraredDescription );

    // To Reserve Infrared Frame Buffer
    infraredBuffer.resize( infraredWidth * infraredHeight );

    signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
    signal_PointXYZI = createSignal<signal_Kinect2_PointXYZI>();
    signal_PointXYZRGB = createSignal<signal_Kinect2_PointXYZRGB>();
    signal_PointXYZRGBA = createSignal<signal_Kinect2_PointXYZRGBA>();
}

Kinect2Grabber::~Kinect2Grabber() throw()
{
    stop();

    disconnect_all_slots<signal_Kinect2_PointXYZ>();
    disconnect_all_slots<signal_Kinect2_PointXYZI>();
    disconnect_all_slots<signal_Kinect2_PointXYZRGB>();
    disconnect_all_slots<signal_Kinect2_PointXYZRGBA>();

    thread.join();

    // End Processing
    if( sensor ){
        sensor->Close();
    }
    SafeRelease( sensor );
    SafeRelease( mapper );
    SafeRelease( colorSource );
    SafeRelease( colorReader );
    SafeRelease( depthSource );
    SafeRelease( depthReader );
    SafeRelease( infraredSource );
    SafeRelease( infraredReader );
}

void Kinect2Grabber::start()
{
    // Open Color Frame Reader
    result = colorSource->OpenReader( &colorReader );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IColorFrameSource::OpenReader()" );
    }

    // Open Depth Frame Reader
    result = depthSource->OpenReader( &depthReader );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IDepthFrameSource::OpenReader()" );
    }

    // Open Infrared Frame Reader
    result = infraredSource->OpenReader( &infraredReader );
    if( FAILED( result ) ){
        throw std::exception( "Exception : IInfraredFrameSource::OpenReader()" );
    }

    running = true;

    thread = boost::thread( &Kinect2Grabber::threadFunction, this );
}

void Kinect2Grabber::stop()
{
    boost::unique_lock<boost::mutex> lock( mutex );

    quit = true;
    running = false;

    lock.unlock();
}

bool Kinect2Grabber::isRunning() const
{
    boost::unique_lock<boost::mutex> lock( mutex );

    return running;

    lock.unlock();
}

bool Kinect2Grabber::isAvailable() const
{
    boost::unique_lock<boost::mutex> lock( mutex );

    return available;

    lock.unlock();
}

std::string Kinect2Grabber::getName() const
{
    return std::string( "Kinect2Grabber" );
}

float Kinect2Grabber::getFramesPerSecond() const
{
    return 30.0f;
}

void Kinect2Grabber::threadFunction()
{
    while( !quit ){
        boost::unique_lock<boost::mutex> lock( mutex );

        // Acquire Latest Color Frame
        IColorFrame* colorFrame = nullptr;
        result = colorReader->AcquireLatestFrame( &colorFrame );
        if( SUCCEEDED( result ) ){
            // Retrieved Color Data
            result = colorFrame->CopyConvertedFrameDataToArray( colorBuffer.size() * sizeof( RGBQUAD ), reinterpret_cast<BYTE*>( &colorBuffer[0] ), ColorImageFormat::ColorImageFormat_Bgra );
            if( FAILED( result ) ){
                throw std::exception( "Exception : IColorFrame::CopyConvertedFrameDataToArray()" );
            }
        }
        SafeRelease( colorFrame );

        // Acquire Latest Depth Frame
        IDepthFrame* depthFrame = nullptr;
        result = depthReader->AcquireLatestFrame( &depthFrame );
        if( SUCCEEDED( result ) ){
            // Retrieved Depth Data
            result = depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] );
            if( FAILED( result ) ){
                throw std::exception( "Exception : IDepthFrame::CopyFrameDataToArray()" );
            }
        }
        SafeRelease( depthFrame );

        // Acquire Latest Infrared Frame
        IInfraredFrame* infraredFrame = nullptr;
        result = infraredReader->AcquireLatestFrame( &infraredFrame );
        if( SUCCEEDED( result ) ){
            // Retrieved Infrared Data
            result = infraredFrame->CopyFrameDataToArray( infraredBuffer.size(), &infraredBuffer[0] );
            if( FAILED( result ) ){
                throw std::exception( "Exception : IInfraredFrame::CopyFrameDataToArray()" );
            }
        }
        SafeRelease( infraredFrame );

        lock.unlock();

        if( signal_PointXYZ->num_slots() > 0 ){
            signal_PointXYZ->operator()( convertDepthToPointXYZ( &depthBuffer[0] ) );
        }

        if( signal_PointXYZI->num_slots() > 0 ){
            signal_PointXYZI->operator()( convertInfraredDepthToPointXYZI( &infraredBuffer[0], &depthBuffer[0] ) );
        }

        if( signal_PointXYZRGB->num_slots() > 0 ){
            signal_PointXYZRGB->operator()( convertRGBDepthToPointXYZRGB( &colorBuffer[0], &depthBuffer[0] ) );
        }

        if( signal_PointXYZRGBA->num_slots() > 0 ){
            signal_PointXYZRGBA->operator()( convertRGBADepthToPointXYZRGBA( &colorBuffer[0], &depthBuffer[0] ) );
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect2Grabber::convertDepthToPointXYZ( UINT16* depthBuffer )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

    cloud->width = static_cast<uint32_t>( depthWidth );
    cloud->height = static_cast<uint32_t>( depthHeight );
    cloud->is_dense = false;

    cloud->points.resize( cloud->height * cloud->width );

    pcl::PointXYZ* pt = &cloud->points[0];
    for( int y = 0; y < depthHeight; y++ ){
        for( int x = 0; x < depthWidth; x++, pt++ ){
            pcl::PointXYZ point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
            UINT16 depth = depthBuffer[y * depthWidth + x];

            // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            mapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );

                point.x = cameraSpacePoint.X;
                point.y = cameraSpacePoint.Y;
                point.z = cameraSpacePoint.Z;


            *pt = point;
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Kinect2Grabber::convertInfraredDepthToPointXYZI( UINT16* infraredBuffer, UINT16* depthBuffer )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZI>() );

    cloud->width = static_cast<uint32_t>( depthWidth );
    cloud->height = static_cast<uint32_t>( depthHeight );
    cloud->is_dense = false;

    cloud->points.resize( cloud->height * cloud->width );

    pcl::PointXYZI* pt = &cloud->points[0];
    for( int y = 0; y < depthHeight; y++ ){
        for( int x = 0; x < depthWidth; x++, pt++ ){
            pcl::PointXYZI point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
            UINT16 depth = depthBuffer[y * depthWidth + x];

            // Setting PointCloud Intensity
            point.intensity = static_cast<float>( infraredBuffer[y * depthWidth + x] );

            // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            mapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
            point.x = cameraSpacePoint.X;
            point.y = cameraSpacePoint.Y;
            point.z = cameraSpacePoint.Z;

            *pt = point;
        }
    }

    return cloud;
}

//USING////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect2Grabber::convertRGBDepthToPointXYZRGB( RGBQUAD* colorBuffer, UINT16* depthBuffer )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );

    cloud->width = static_cast<uint32_t>( depthWidth );
    cloud->height = static_cast<uint32_t>( depthHeight );
    cloud->is_dense = false;

    cloud->points.resize( cloud->height * cloud->width );

    pcl::PointXYZRGB* pt = &cloud->points[0];
    for( int y = 0; y < depthHeight; y++ ){
        for( int x = 0; x < depthWidth; x++, pt++ ){
            pcl::PointXYZRGB point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
            UINT16 depth = depthBuffer[y * depthWidth + x];

            // Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
            ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
            mapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );
            int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );
            int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
                point.b = color.rgbBlue;
                point.g = color.rgbGreen;
                point.r = color.rgbRed;
            }

            // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            mapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
       //      //A LIMIT IN THE DEPTH MUST BE SET WHEN WE KNOW THE OPERATING DISTANCE (Kinect to platform)
                if (cameraSpacePoint.Z >= 0.2)
                {
                   if (cameraSpacePoint.Y <= 1.2 && cameraSpacePoint.Y >= -1.2)
                   {
                       if (cameraSpacePoint.X <= 0.6 && cameraSpacePoint.X >= -0.6 ){
                       point.x = cameraSpacePoint.X;
                       point.y = cameraSpacePoint.Y;
                       point.z = cameraSpacePoint.Z;

                       *pt = point;}
                   }
                }

            }
        }
    }

    return cloud;
}
//endUSING///////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Kinect2Grabber::convertRGBADepthToPointXYZRGBA( RGBQUAD* colorBuffer, UINT16* depthBuffer )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA>() );

    cloud->width = static_cast<uint32_t>( depthWidth );
    cloud->height = static_cast<uint32_t>( depthHeight );
    cloud->is_dense = false;

    cloud->points.resize( cloud->height * cloud->width );

    pcl::PointXYZRGBA* pt = &cloud->points[0];
    for( int y = 0; y < depthHeight; y++ ){
        for( int x = 0; x < depthWidth; x++, pt++ ){
            pcl::PointXYZRGBA point;

            DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
            UINT16 depth = depthBuffer[y * depthWidth + x];

            // Coordinate Mapping Depth to Color Space, and Setting PointCloud RGBA
            ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
            mapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );
            int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );
            int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
                point.b = color.rgbBlue;
                point.g = color.rgbGreen;
                point.r = color.rgbRed;
                point.a = color.rgbReserved;
            }

            // Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
            CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
            mapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                point.x = cameraSpacePoint.X;
                point.y = cameraSpacePoint.Y;
                point.z = cameraSpacePoint.Z;
            }

            *pt = point;
        }
    }

    return cloud;
}
