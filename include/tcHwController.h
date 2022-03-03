/**                                                                               
 * @file tcHwController.h                                                         
 * @author Anthony Cianfrocco                                                     
 * @email afc4328@rit.edu                                                         
 *                                                                                
 * @description This class handles publishing and subscribing to messages         
 *              related to navigation from the robot.                             
 */                                                                               
                                                                                  
#ifndef INCLUDE_TCHWCONTROLLER_H_                                                 
#define INCLUDE_TCHWCONTROLLER_H_

#include "ros/ros.h"                                                              
#include "geometry_msgs/Twist.h"                                                  
#include "nav_msgs/Odometry.h"                                                    
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <thread>                                                                 
#include <mutex>                                                                  
#include <memory> 

class tcHwController
{
public:
    /**
     * Ctor
     */
    tcHwController(int argc, char* argv[]);

    /**
     * Dtor
     */
    ~tcHwController();

    /**
     * Call back for the odom msg
     */
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& arcOdomMsg);

    /**                                                                           
     * Get the most recent odom rcv'd from the robot                              
     */                                                                           
    nav_msgs::Odometry GetMostRecentOdom();

    /**                                                                           
     * Shutdown the publisher and subscriber                                      
     */                                                                           
    void Shutdown(); 

    /**                                                                           
     * This is a threaded function that continuously sends /r1/cmd_vel            
     * commands to the robot at a 10Hz frequency. The velocity values it          
     * uses are the member variables mrLinearX and mrAngularZ. Our robot          
     * only can move in these two directions which is why the other velocity      
     * values are left as zero.                                                   
     */                                                                           
    void PublishCmdVelMsg();

    /**                                                                           
     * Call back for the LaserScan msg from the kinnect                           
     */                                                                           
    void KinnectCallback(const sensor_msgs::LaserScan::ConstPtr& arcLaerScanMsg);

    /**                                                                           
     * Get the most recent LaserScan recv'd from the robot's kinnect              
     */                                                                           
    sensor_msgs::LaserScan GetMostRecentKinnectScan();

    /**
     *
     */
    void SetIsLocalized(bool abIsLocalized);

    /**
     *
     */
    void SetLocOdom(const float arX, const float arY, 
            const float arAngularZ);

    /**
     * Call back for the PointCloudMsg from the kinect
     */
    void PointCloudCallback(
            const sensor_msgs::PointCloud2::ConstPtr& arcPointCloudMsg);

    /**
     * Get the most recent PointCloud recv'd from the robot's kinect
     */
    sensor_msgs::PointCloud2 GetMostRecentPointCloud();

private:

    /**
     * Checks if a sigint happened.
     */
    bool CheckSigInt();

    /**
     * Node handle for this node. Main access point to communications within the 
     * ROS system.
     */
    ros::NodeHandle mcNodeHandle;

    /**
     * Subscriber for the /r1/odom topic
     */
    ros::Subscriber mcOdomSub;

    /**
     * Frequency to publish messages to ROS system
     */
    ros::Rate mcLoopRate;

    /**
     * Most recent odom recv'd from the robot
     */
    nav_msgs::Odometry mcCurrentOdom;

    /**
     * Used on SigInt
     */
    bool mbFinishFlag;

    /**
     * Protects mbFinishFlag
     */
    std::mutex mcSigIntMutex;

    /**                                                                           
     * Thread for publisher                                                       
     */                                                                           
    std::thread mcPubThread;

    /**                                                                           
     * Mutex to protect the most current position value of the robot              
     */                                                                           
    std::mutex mcPoseMutex;

    /**                                                                           
     * Subscriber for the /r1/kinect_laser/scan topic                             
     */                                                                           
    ros::Subscriber mcKinnectSub;

    /**
     * Mutex to protect the most current laser scan data from the robot's kinnect
     */
    std::mutex mcLaserScanMutex;

    /**                                                                           
     * Most recent kinnect data recv'd from the robot                             
     */                                                                           
    sensor_msgs::LaserScan mcCurrentKinnectScan;

    /**
     * Subscriber for the /r1/camera/depth/points topic
     */
    ros::Subscriber mcPointCloudSub; 
    
    /**
     * Mutex to protect the most recent point cloud data
     */
    std::mutex mcPointCloudMutex;

    /**
     * Most recent point cloud generated from Kinect scan
     */
    sensor_msgs::PointCloud2 mcCurrentPointCloud;

    /**
     * Localized flag publisher
     */
    ros::Publisher mcLocalizedFlagPub;

    /**
     * Localized Odometry publisher
     */
    ros::Publisher mcLocalizedOdomPub;

    /**
     * Mutex for localized data
     */
    std::mutex mcLocMutex;

    float mrLocalizedXMtrs;
    float mrLocalizedYMtrs;
    float mrLocalizedZ;
    bool mbIsLocalized;
};

#endif /* INCLUDE_TCHWCONTROLLER_H_ */
