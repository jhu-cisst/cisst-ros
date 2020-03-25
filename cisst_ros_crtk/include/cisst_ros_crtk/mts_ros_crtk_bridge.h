/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mts_ros_crtk_bridge_h
#define _mts_ros_crtk_bridge_h

// cisst include
#include <cisstMultiTask/mtsTaskFromSignal.h>

class mtsROSBridge;

// ROS
#include <ros/ros.h>


class mts_ros_crtk_bridge: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    /*!  Constructor using an existing ros::NodeHandle.  This
      construtor creates 2 mtsROSBridge using a high periodicity to
      reduce latency.  The first one, m_subscribers_bridge is used
      for all the subscribers added to this CRTK bridge.  It
      performs the ros::spinOnce().  The second one, m_events_bridge
      is used for all events and messages coming from all the
      provided interface added to this bridge. */
    mts_ros_crtk_bridge(const std::string & component_name,
                        ros::NodeHandle * node_handle);

    ~mts_ros_crtk_bridge();

    inline void Configure(const std::string & CMN_UNUSED(filename) = "") {};

    inline void Startup(void) {};
    void Run(void);
    inline void Cleanup(void) {};

    /*! This method will look at all the provided functions and events
      that match the CRTK convention and automatically create the
      corresponding ROS topics (publishers and subscribers).  This
      method also adds ROS topics for a few non-CRTK including all
      events using prmEventButton, errors/warnings and interval
      statistics. */
    void bridge_interface_provided(const std::string & component_name,
                                   const std::string & interface_name,
                                   const double publish_period_in_seconds,
                                   const std::string & ros_namespace);

    /*! Same method but used the name of the provided interface as ROS
      namespace. */
    inline void bridge_interface_provided(const std::string & component_name,
                                          const std::string & interface_name,
                                          const double publish_period_in_seconds) {
        bridge_interface_provided(component_name, interface_name,
                                  publish_period_in_seconds, interface_name);
    }

protected:
    //! ros node
    ros::NodeHandle * m_node_handle_ptr = nullptr;

    mtsROSBridge * m_subscribers_bridge = nullptr;
    mtsROSBridge * m_events_bridge = nullptr;
    mtsROSBridge * m_stats_bridge = nullptr;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mts_ros_crtk_bridge);

#endif // _mts_ros_crtk_bridge_h
