/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mts_ros_crtk_bridge_required_h
#define _mts_ros_crtk_bridge_required_h

// cisst include
#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsDelayedConnections.h>
#include <cisst_ros_crtk/cisst_ros_crtk.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

class mts_ros_crtk_bridge_required: public mtsROSBridge
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    /*!  Constructor using an existing ros::NodeHandle.  This
      construtor creates 2 mtsROSBridge using a high periodicity to
      reduce latency.  The first one, m_subscribers_bridge is used for
      all the subscribers added to this CRTK bridge.  It performs the
      ros::spinOnce().  The second one, m_events_bridge is used for
      all events and messages coming from all the provided interface
      added to this bridge.  The period in seconds is for the bridge
      management itself, i.e. create new bridges dynamically.  It
      doesn't impact the ROS publish rates.  There is no reason to
      change the default for most applications. */
    mts_ros_crtk_bridge_required(const std::string & _component_name,
                                 ros::NodeHandle * _node_handle,
                                 const double _period_in_seconds = 0.1 * cmn_ms);
    mts_ros_crtk_bridge_required(const mtsTaskPeriodicConstructorArg & arg);

    ~mts_ros_crtk_bridge_required();

    void init(void); // called by all ctors
    void Configure(const std::string & _json_file) override;
    void ConfigureJSON(const Json::Value & _json_config);

    inline void Startup(void) {};
    void Run(void);
    inline void Cleanup(void) {};

    /*! This method will look at all the required functions and event
      handlers that match the CRTK convention and automatically create
      the corresponding ROS topics (publishers and subscribers).  This
      method also adds ROS topics for a few non-CRTK topics including
      all events using prmEventButton, errors/warnings, and jacobians.
      All commands and events using a '/' in their names are also
      parsed to check if they match the CRTK covention,
      i.e. `local/measured_cp` will also be automatically bridged. */
    void bridge_interface_required(const std::string & _component_name,
                                   const std::string & _interface_name,
                                   const std::string & _ros_namespace);

    void populate_interface_provided(const std::string & _interface_name,
                                     const std::string & _ros_namespace,
                                     const std::vector<std::string> & _void_commands,
                                     const std::vector<std::string> & _write_commands,
                                     const std::vector<std::string> & _read_commands,
                                     const std::vector<std::string> & _write_events);

    /*! Connect all components created and used so far. */
    inline virtual void Connect(void) {
        m_connections.Connect();
    }

protected:
    mtsDelayedConnections m_connections;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mts_ros_crtk_bridge_required);

#endif // _mts_ros_crtk_bridge_required_h
