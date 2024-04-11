#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "rslidar_laserscan/rslidar_laserscan.h"

namespace rslidar_laserscan
{
    class LaserScanNodelet : public nodelet::Nodelet
    {
    private:
        boost::shared_ptr<RslidarLaserScan> node_;
        virtual void onInit()
        {
            node_.reset(new RslidarLaserScan(getNodeHandle(), getPrivateNodeHandle()));
        }

    public:
        LaserScanNodelet() {}
        ~LaserScanNodelet() {}
    };
} // namespace rslidar_laserscan

PLUGINLIB_EXPORT_CLASS(rslidar_laserscan::LaserScanNodelet, nodelet::Nodelet);