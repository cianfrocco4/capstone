/**
 * @file tcLocMgr.h
 * @author Anthony Cianfrocco
 * @email afc4328@rit.edu
 *
 * @description Manages the localization of the robot.
 *
 */

#ifndef INCLUDE_TCLOCMGR_H
#define INCLUDE_TCLOCMGR_H

#include <vector>
#include <string>
#include <memory>
#include <thread>

#include "CommonTypes.h"
#include "tcMapMgr.h"

class tcLocMgr
{
public:
    /**
     * CTOR
     */
    tcLocMgr(const std::vector<CommonTypes::tsPose> &arcStartingPoseVec,
             const std::string &arcMapFilename, int argc, char *argv[]);

    /**
     * DTOR
     */
    ~tcLocMgr() = default;

    /**
     * Runs the localization algorithm.
     */
    void Run(int argc, char *argv[]);

private:
    /**
     * Threaded function that will do the localization.
     */
    void Localize();

    /**
     * Initialize the particles to be used for localization.
     */
    void InitializeParticles(
            const std::vector<CommonTypes::tsPose> &arcStartingPoseVec);

    /**
     * The prediction phase of the localization.
     */
    void Predict();

    /**
     * The update phase of the localization.
     */
    void Update();

    /**
     * The resample phase of the localization.
     */
    void Resample();

    /**
     * Normalize the particles so their probabilites sum to 1.
     */
    void NormalizeParticles();

    /**
     * Generate new particles based on current ones with high prob.
     */
    void GenerateNewParticles(const float arXPercent);

    /**
     * Returns a random number.
     */
    float GetRand(float arVal);

    /** MEMBER DATA */

    /**
     * Handle for MapMgr object.
     */
    tcMapMgr mcMapMgr;    

    /**
     * Thread that will handle localization.
     */
    std::thread mcLocalizeThread;

    /**
     * Vector of all of the particles.
     */
    std::vector<CommonTypes::tsParticle> mcParticleVec;

    /**
     * Current odometry.
     */
    nav_msgs::Odometry mcCurrOdom;

    /**
     * Odometry of the last step.
     */
    nav_msgs::Odometry mcPrevOdom;

    /**
     * Current kinect laser scan data. TODO - change to kinect depth img data
     */
    sensor_msgs::LaserScan mcCurrLaserScan;

    /**
     * Current 3D depth point cloud generated from Kinect scan
     */
    sensor_msgs::PointCloud2 mcCurrPointCloud;
    /**
     * HwController handle.
     */
    tcHwController mcHwCtrl;
};

#endif /* INCLUDE_TCLOCMGR_H */
