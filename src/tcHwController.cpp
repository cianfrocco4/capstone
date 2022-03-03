/**                                                                               
 * @file tcHwController.cpp                                                       
 * @author Anthony Cianfrocco                                                     
 * @email afc4328@rit.edu                                                         
 *                                                                                
 * @description See header file.                                                  
 */                                                                               
                                                                                  
#include "tcHwController.h"                                                       
                                                                                  
#include <iostream>
#include "std_msgs/Bool.h"

tcHwController::tcHwController(int argc, char* argv[]) :
    mcLoopRate(10),
    mbFinishFlag(false),
    mrLocalizedXMtrs(0),
    mrLocalizedYMtrs(0),
    mrLocalizedZ(0),
    mbIsLocalized(false)
{
    // Create the ros node
    ros::init(argc,argv,"localization-3d");
    mcNodeHandle = ros::NodeHandle();        

    mcOdomSub = mcNodeHandle.subscribe("/r1/odom", 1, 
            &tcHwController::OdomCallback, this);

    ROS_INFO("Setup OdomSub complete");

    // Subscriber for the kinnect "laser"                                         
    mcKinnectSub = mcNodeHandle.subscribe("/r1/kinect_laser/scan",              
                                            1,                                    
                                            &tcHwController::KinnectCallback,        
                                            this);                                
                                                                                  
    ROS_INFO("Setup KinectSub complete"); 

    // Subscriber for the Kinect point cloud topic
    mcPointCloudSub = mcNodeHandle.subscribe("/camera/depth/points",
                                               1,
                                               &tcHwController::PointCloudCallback,
                                               this);

    ROS_INFO("Setup PointCloudSub complete");

    mcLocalizedFlagPub = 
        mcNodeHandle.advertise<std_msgs::Bool>("/localizedFlag", 1000, true);

    ROS_INFO("Setup LocalizedFlagPub complete");

    mcLocalizedOdomPub = 
        mcNodeHandle.advertise<nav_msgs::Odometry>(
                "/localizedOdom", 1000, true);

    ROS_INFO("Setup OdomPub complete");

    mcPubThread = std::thread(&tcHwController::PublishCmdVelMsg, this);
}

tcHwController::~tcHwController()
{
    mcPubThread.join();
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::OdomCallback(const nav_msgs::Odometry::ConstPtr& arcOdomMsg)
{
//    ROS_INFO("Reached OdomCallback()");
    std::lock_guard<std::mutex> lcLock(mcPoseMutex);
    mcCurrentOdom.pose.pose = arcOdomMsg->pose.pose;
    mcCurrentOdom.twist.twist = arcOdomMsg->twist.twist;
    mcCurrentOdom.header.stamp = arcOdomMsg->header.stamp;
/*    ROS_INFO("Curr Odom = [%f, %f]",
            mcCurrentOdom.pose.pose.position.x,
            mcCurrentOdom.pose.pose.position.y);
*/
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::PublishCmdVelMsg()
{
    // Continues until ctrl-c is caught
    while(ros::ok() && !CheckSigInt())
    {
        // Lock mutex to get the most recent linear X and angular Z values
        std::unique_lock<std::mutex> lcLock(mcLocMutex);
        nav_msgs::Odometry lcOdomMsg{};
        lcOdomMsg.pose.pose.position.x = mrLocalizedXMtrs;
        lcOdomMsg.pose.pose.position.y = mrLocalizedYMtrs;
        lcOdomMsg.pose.pose.orientation.z = mrLocalizedZ;

        std_msgs::Bool lcBoolMsg;
        lcBoolMsg.data = mbIsLocalized;
        // release lock manually
        lcLock.unlock();

        mcLocalizedOdomPub.publish(lcOdomMsg);
        mcLocalizedFlagPub.publish(lcBoolMsg);

        ros::spinOnce();

        mcLoopRate.sleep();
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
nav_msgs::Odometry
tcHwController::GetMostRecentOdom()
{
    std::lock_guard<std::mutex> lcLock(mcPoseMutex);
    return this->mcCurrentOdom;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::Shutdown()
{
    std::cout << "HwController:Shutdown()\n";
    // Tell the pub thread to stop and wait for it to finish
    {
        std::lock_guard<std::mutex> lcLock(mcSigIntMutex);
        mbFinishFlag = true;
    }
    mcPubThread.join();

    std::cout << "Publisher thread has finished\n";
}

///////////////////////////////////////////////////////////////////////////////   
// See Header File                                                                
///////////////////////////////////////////////////////////////////////////////   
bool                                                                              
tcHwController::CheckSigInt()                                                     
{                                                                                 
    std::lock_guard<std::mutex> lcLock(mcSigIntMutex);                            
    return mbFinishFlag;                                                          
}

///////////////////////////////////////////////////////////////////////////////   
// See Header File                                                                
///////////////////////////////////////////////////////////////////////////////   
void                                                                              
tcHwController::KinnectCallback(const sensor_msgs::LaserScan::ConstPtr& arcLaserScanMsg)
{                                                                                 
//    ROS_INFO("Reached KinnectCallback()");                                      
    std::lock_guard<std::mutex> lcLock(mcLaserScanMutex);                         
                                                                                  
    // start angle of the scan [rad]                                              
    mcCurrentKinnectScan.angle_min = arcLaserScanMsg->angle_min;                  
                                                                                  
    // end angle of the scan [rad]                                                
    mcCurrentKinnectScan.angle_max = arcLaserScanMsg->angle_max;                  
                                                                                  
    // angular distance between measurements [rad]                                
    mcCurrentKinnectScan.angle_increment = arcLaserScanMsg->angle_increment;      
                                                                                  
    // time between measurements [seconds] - if your scanner is moving, this      
    //                                       will be used in interpolating position
    //                                       of 3d points                         
    mcCurrentKinnectScan.time_increment = arcLaserScanMsg->time_increment;        
                                                                                  
    // time between scans [seconds]                                               
    mcCurrentKinnectScan.scan_time = arcLaserScanMsg->scan_time;                  
                                                                                  
    // minimum range value [m]                                                    
    mcCurrentKinnectScan.range_min = arcLaserScanMsg->range_min;                  
    // maximum range value [m]                                                    
    mcCurrentKinnectScan.range_max = arcLaserScanMsg->range_max;                  
                                                                                  
    // range data [m] (Note: values < range_min or > range_max should be discarded)
    // range value of "inf" means the laser did not detect anything within range  
    mcCurrentKinnectScan.ranges = arcLaserScanMsg->ranges;                        
                                                                                  
    // intensity data [device-specific units].  If your                           
    //                     device does not provide intensities, please leave      
    //                     the array empty.                                       
    mcCurrentKinnectScan.intensities = arcLaserScanMsg->intensities;              
                                                                                  
    mcCurrentKinnectScan.header.stamp = arcLaserScanMsg->header.stamp; 

/*
    ROS_INFO("KinnectCallback: Ranges.size() = %li, angle_min = %f, angle_max = %f, angle_increment = %f time_increment = %f, scan_time = %f, range_min = %f, range_max = %f",
            mcCurrentKinnectScan.ranges.size(),
        mcCurrentKinnectScan.angle_min,
            mcCurrentKinnectScan.angle_max,
            mcCurrentKinnectScan.angle_increment,
            mcCurrentKinnectScan.time_increment,
            mcCurrentKinnectScan.scan_time,
            mcCurrentKinnectScan.range_min,
            mcCurrentKinnectScan.range_max);
*/
    std::stringstream lcSs;
    for(auto x : mcCurrentKinnectScan.ranges)
    {
        lcSs << std::to_string(x) << ", ";
    }
//    ROS_INFO("Ranges = %s", lcSs.str().c_str());

    std::stringstream ss;
    for(auto x : mcCurrentKinnectScan.intensities)
    {
        ss << std::to_string(x) << ", ";
    }
//    ROS_INFO("Intensities = %s", ss.str().c_str());
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
sensor_msgs::LaserScan
tcHwController::GetMostRecentKinnectScan()
{
    std::lock_guard<std::mutex> lcLock(mcLaserScanMutex);
    return mcCurrentKinnectScan;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::SetIsLocalized(bool abIsLocalized)
{
    std::lock_guard<std::mutex> lcLock(mcLocMutex);
    mbIsLocalized = abIsLocalized;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::SetLocOdom(const float arX, const float arY,
        const float arAngularZ)
{
    std::lock_guard<std::mutex> lcLock(mcLocMutex);
    mrLocalizedXMtrs = arX;
    mrLocalizedYMtrs = arY;
    mrLocalizedZ = arAngularZ;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::PointCloudCallback(
        const sensor_msgs::PointCloud2::ConstPtr& arcPointCloudMsg)
{
    std::lock_guard<std::mutex> lcLock(mcPointCloudMutex);
    mcCurrentPointCloud.header = arcPointCloudMsg->header;
    mcCurrentPointCloud.height = arcPointCloudMsg->height;
    mcCurrentPointCloud.width = arcPointCloudMsg->width;
    mcCurrentPointCloud.fields = arcPointCloudMsg->fields;
    mcCurrentPointCloud.is_bigendian = arcPointCloudMsg->is_bigendian;
    mcCurrentPointCloud.point_step = arcPointCloudMsg->point_step;
    mcCurrentPointCloud.row_step = arcPointCloudMsg->row_step;
    mcCurrentPointCloud.data = arcPointCloudMsg->data;
    mcCurrentPointCloud.is_dense = arcPointCloudMsg->is_dense;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
sensor_msgs::PointCloud2
tcHwController::GetMostRecentPointCloud()
{
    std::lock_guard<std::mutex> lcLock(mcPointCloudMutex);
    return mcCurrentPointCloud;  
}

