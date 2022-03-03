/**
 * @file tcLocMgr.cpp
 * @author Anthony Cianfrocco
 * @email afc4328@rit.edu
 *
 * @description See header file.
 *
 */

#include <tcLocMgr.h>

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
tcLocMgr::tcLocMgr(
        const std::vector<CommonTypes::tsPose> &arcStartingPoseVec,
        const std::string &arcMapFilename, 
        int argc, 
        char *argv[]) :
    mcMapMgr(arcStartingPoseVec, arcMapFilename, argc, argv),
    mcHwCtrl(argc, argv)
{
    // Initalize particles
    InitializeParticles(arcStartingPoseVec);
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Run(int argc, char *argv[])
{

    // Start a thread that will do localization
    mcLocalizeThread = std::thread(&tcLocMgr::Localize, this);

    // Launch the GUI
    // NOTE - this will take control of the main thread
    mcMapMgr.Run(argc, argv);
    
    mcLocalizeThread.join();
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Localize()
{
    // Draw the intial particles
    mcMapMgr.DrawParticlesFromMeters(mcParticleVec);

    // init curr robot sensor data
    mcCurrOdom = mcHwCtrl.GetMostRecentOdom();
    mcCurrLaserScan = mcHwCtrl.GetMostRecentKinnectScan(); // TODO - change to depth img
    mcCurrPointCloud = mcHwCtrl.GetMostRecentPointCloud();
    mcPrevOdom = mcCurrOdom;

    while(ros::ok())
    {
        // Undraw the particles on the map before doing prediction
        mcMapMgr.UndrawParticlesFromMeters(mcParticleVec);

        // 1) Predict()
        Predict();

        // Redraw the predicted particles
        mcMapMgr.DrawParticlesFromMeters(mcParticleVec);

        // 2) Update();
        Update();

        mcMapMgr.UndrawParticlesFromMeters(mcParticleVec);

        // 3) Resample();
        Resample(); 

        mcMapMgr.DrawParticlesFromMeters(mcParticleVec);
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::InitializeParticles(
        const std::vector<CommonTypes::tsPose> &arcStartingPoseVec)
{
    ROS_INFO("InitializeParticles()");
    // TODO get x/y bounds from tcMapMgr

    for(auto lsPose : arcStartingPoseVec)
    {
        const int lnParticleWidth = 30;
        const int lnParticleHeight = 30;
        const float lrParticleOffsetMtrs = 0.5; 

        float lrXmtrs = lsPose.mrX - (lnParticleWidth * (lrParticleOffsetMtrs / 2.0));
        while(lrXmtrs < lsPose.mrX + (lnParticleWidth * (lrParticleOffsetMtrs / 2.0)))
        {
            float lrYmtrs = lsPose.mrY - (lnParticleHeight * (lrParticleOffsetMtrs / 2.0));
            while(lrYmtrs < lsPose.mrY + (lnParticleHeight * (lrParticleOffsetMtrs / 2.0)))
            {
                // TODO if y is in bounds

                // Set the new particles pose
                CommonTypes::tsPose lsNewPose(lrXmtrs, lrYmtrs, lsPose.mrTheta);
                
                // Set the new particles probability
                CommonTypes::tsParticle lsNewParticle(lsNewPose, 0.5);

                // Add the new particle to the vector
                mcParticleVec.push_back(lsNewParticle);

/*                ROS_INFO_STREAM("Adding particle " << std::to_string(lrXmtrs) << 
                    ", " << std::to_string(lrYmtrs) << ", " << 
                    std::to_string(lsPose.mrTheta) << ", " << 
                    std::to_string(mcParticleVec.size()) << 
                    std::endl);
*/
                lrYmtrs += lrParticleOffsetMtrs;
            }

            lrXmtrs += lrParticleOffsetMtrs; 
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Predict()
{
    // Prediction Phase
    //  1) get motion since last cycle
    //  2) convert to forward delta and angular delta
    //  3) add deltas to each particle plus noise
    //  4) update the map after prediction phase

    // 1) Get motion since last cycle
    //      - Compare previous odom to current odom
    float lrForwardDelta, lrCurrHeading, lrHeadingDelta;

    const float lrPrevHeading = 2 * atan2(mcPrevOdom.pose.pose.orientation.z,
            mcPrevOdom.pose.pose.orientation.w);


    // loop until either the forward or angular delta has changed "enough"
    do
    {
        // Get the current odom
        mcCurrOdom = mcHwCtrl.GetMostRecentOdom();
        
        // Get the current laser scan to be consistent with odom
        mcCurrLaserScan = mcHwCtrl.GetMostRecentKinnectScan();

        lrForwardDelta = std::sqrt(
                std::pow(mcPrevOdom.pose.pose.position.x -
                         mcCurrOdom.pose.pose.position.x, 2)
                +
                std::pow(mcPrevOdom.pose.pose.position.y -
                         mcCurrOdom.pose.pose.position.y, 2)
                );

        lrCurrHeading = 2 * atan2(mcCurrOdom.pose.pose.orientation.z,
                                  mcCurrOdom.pose.pose.orientation.w);

        lrHeadingDelta = std::abs(lrCurrHeading - lrPrevHeading);
    } while(ros::ok() && lrForwardDelta < 0.1 && lrHeadingDelta < 0.05);
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Update()
{
    // TODO

    // Update Phase
    //  1) For each particle
    //      a) compute expected sensor readings and pose
    //      b) particles in walls are not possible
    //      c) compute prob of particle based on actual and expected sensor 
    //         values
    //      d) Multiply previous particle prob by new one

    for(int lnI = 0; lnI < mcParticleVec.size(); lnI++)
    {
        CommonTypes::tsParticle lsPart = mcParticleVec[lnI];

        // check if particle is in a obstacle
        if(mcMapMgr.IsPoseInMtrsValid(lsPart.msPose) == false)
        {
            mcParticleVec[lnI].mrProb = 0;
        }
        else
        {
            // Vector of distances away from particle at angles 0.4, 0.2, 0, -0.2
            // and -0.4 respectively
            std::vector<float> lcDistVec;

            // Increment for angle to check in scan
            const float lrInc = 0.025; // about 1 degree

            float lrAngleOffset = mcCurrLaserScan.angle_min;//-0.4; // radians
            while(lrAngleOffset <= mcCurrLaserScan.angle_max)
            {
                
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::NormalizeParticles()
{
    float lrSum = 0;
    for(auto lsP : mcParticleVec)
    {
        lrSum += lsP.mrProb;
    }

    for(int lnI = 0; lnI < mcParticleVec.size(); ++lnI)
    {
        mcParticleVec[lnI].mrProb = mcParticleVec[lnI].mrProb / lrSum; 
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
float
tcLocMgr::GetRand(float arVal)
{
    float lrRand = ((float) std::rand() / RAND_MAX) * arVal * 2;
    if(lrRand < arVal)
        lrRand *= -1;
    else
        lrRand -= arVal;

    return lrRand;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::GenerateNewParticles(const float arXPercent)
{
    float lrN = 1 - arXPercent;
    int lnNewSize = lrN / mcParticleVec.size();
    int lnNumberToGenerate = lnNewSize - mcParticleVec.size();

    while(lnNumberToGenerate > 0)
    {
        int lnSize = mcParticleVec.size();
        const float lrForwardNoiseScalar = 1; // in pixels
        const float lrAngularNoiseScalar = 0;
        for(int lnI = 0; lnI < (lnSize / 4) && lnI < lnNumberToGenerate; lnI++)
        {
            auto lsP = mcParticleVec[lnI];
            CommonTypes::tsParticle lsNew(CommonTypes::tsPose(0, 0, 0), 0);
            float lrRand = GetRand(lrForwardNoiseScalar); // get rand number
            lsNew.msPose.mrX = lsP.msPose.mrX + lrRand; // adding noise
            lrRand = GetRand(lrForwardNoiseScalar);
            lsNew.msPose.mrY = lsP.msPose.mrY + lrRand;
            lrRand = GetRand(lrAngularNoiseScalar);
            lsNew.msPose.mrTheta = lsP.msPose.mrTheta + lrRand;
            lsNew.mrProb = lsP.mrProb;

            mcParticleVec.push_back(lsNew);
            lnNumberToGenerate -= 1;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Resample()
{
    // Resample
    //  a) Normalize all prob values
    //  b) check if particles are < theshold value and get rid of it if yes
    //  c) generate new particles if removed over 1/2 of original
    //      - duplicate the ones with higher probability

    NormalizeParticles();

    std::sort(mcParticleVec.begin(), mcParticleVec.end(),
            [this](CommonTypes::tsParticle const &lhs,
                CommonTypes::tsParticle const &rhs)
            {
                return lhs.mrProb > rhs.mrProb;
            }
            );

    // Remove worst X percent
    float lrXPercent = 0.2; 

    int lnBeginBadPart = 
        mcParticleVec.size() - (mcParticleVec.size() * lrXPercent);
    
    mcParticleVec.erase(mcParticleVec.begin() + lnBeginBadPart,
            mcParticleVec.end());

    GenerateNewParticles(lrXPercent);
}

