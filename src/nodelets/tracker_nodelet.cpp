#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include <iostream>

#include "inside_out_tracker.h"

namespace inside_out_tracker {

    class TrackerNodelet: public nodelet::Nodelet {
    public:
        TrackerNodelet(){}

    private:
        void onInit() {
            inside_out_tracker.reset(new InsideOutTracker(getNodeHandle(), getPrivateNodeHandle()));
        }

        boost::shared_ptr<InsideOutTracker> inside_out_tracker;
    };

}

PLUGINLIB_DECLARE_CLASS(hri_planner, TrackerNodelet, inside_out_tracker::TrackerNodelet, nodelet::Nodelet);