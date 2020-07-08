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
#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstMultiTask/mtsDelayedConnections.h>

class mtsROSBridge;

// ROS
#include <ros/ros.h>

const double mts_ros_crtk_bridge_default_publish_period = 10.0 * cmn_ms;
const double mts_ros_crtk_bridge_default_tf_period = 20.0 * cmn_ms;

class mts_ros_crtk_bridge: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

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
    mts_ros_crtk_bridge(const mtsTaskConstructorArg & arg);

    ~mts_ros_crtk_bridge();

    void init(void); // called by all ctors
    void Configure(const std::string & _json_file) override;
    void ConfigureJSON(const Json::Value & _json_config);

    inline void Startup(void) {};
    void Run(void);
    inline void Cleanup(void) {};


    /*! This method will look at all the provided functions and events
      that match the CRTK convention and automatically create the
      corresponding ROS topics (publishers and subscribers).  This
      method also adds ROS topics for a few non-CRTK topics including
      all events using prmEventButton, errors/warnings, jacobians and
      interval statistics.  All commands and events using a '/' in
      their names are also parsed to check if they match the CRTK
      covention, i.e. `local/measured_cp` will also be automatically
      bridged. */
    void bridge_interface_provided(const std::string & _component_name,
                                   const std::string & _interface_name,
                                   const std::string & _ros_namespace,
                                   const double _publish_period_in_seconds
                                   = mts_ros_crtk_bridge_default_publish_period,
                                   const double _tf_period_in_seconds
                                   = mts_ros_crtk_bridge_default_tf_period);

    /*! Same method but used the name of the provided interface as ROS
      namespace. */
    inline void bridge_interface_provided(const std::string & _component_name,
                                          const std::string & _interface_name,
                                          const double _publish_period_in_seconds
                                          = mts_ros_crtk_bridge_default_publish_period,
                                          const double _tf_period_in_seconds
                                          = mts_ros_crtk_bridge_default_tf_period) {
        bridge_interface_provided(_component_name, _interface_name,
                                  _interface_name, // use interface names as ROS namespace
                                  _publish_period_in_seconds,
                                  _tf_period_in_seconds);
    }

    /*! Connect to an interface that will provide the name of newly
     created CRTK interfaces.  The interface provided must have an
     event void called "crtk_interfaces_provided_updated" and a read
     command to find all existing CRTK interfaces called
     "crtk_interfaces_provided" */
    void add_factory_source(const std::string & _component_name,
                            const std::string & _interface_name,
                            const double _publish_period_in_seconds
                            = mts_ros_crtk_bridge_default_publish_period,
                            const double _tf_period_in_seconds
                            = mts_ros_crtk_bridge_default_tf_period);

    /*! Connect all components created and used so far. */
    inline virtual void Connect(void) {
        m_connections.Connect();
    }

    /*! Create and start all components created so for.  This is
      useful if a bridge is dynamically added after the main component
      manager calls for CreateAll and StartAll. */
    void virtual CreateStartAndWait(const double & timeoutInSeconds = 1.0 * cmn_s);

    /*! Access to different bridges to allow users to add custom
      publishers/subscribers */
    inline mtsROSBridge & subscribers_bridge(void) {
        return *m_subscribers_bridge;
    }

    inline mtsROSBridge & events_bridge(void) {
        return *m_events_bridge;
    }

    inline mtsROSBridge & stats_bridge(void) {
        return *m_stats_bridge;
    }

    inline ros::NodeHandle * node_handle_ptr(void) {
        return m_node_handle_ptr;
    }

    /*! Replaces characters that can't be used in ROS namespace by '_'
      (e.g. space, '.', '-') */
    static void clean_namespace(std::string & _ros_namespace);

    /*! Extract the CRTK command from an mts command that might
      include a namespace.  E.g. if the full command is
      "gripper/measured_js", the corresponding crtk command is
      "measures_js" */
    static void get_crtk_command(const std::string & _full_command,
                                 std::string & _crtk_command);

protected:
    //! ros node
    ros::NodeHandle * m_node_handle_ptr = nullptr;

    mtsROSBridge * m_subscribers_bridge = nullptr;
    mtsROSBridge * m_events_bridge = nullptr;
    mtsROSBridge * m_stats_bridge = nullptr;

    mtsDelayedConnections m_connections;
    std::list<std::string> m_new_components;

    //! Explicit list of CRTK commands to bridge
    std::set<std::string> m_bridge_only;
    bool should_be_bridged(const std::string & _command);

    class factory {
    public:
        mts_ros_crtk_bridge * m_bridge;
        double m_publish_period;
        double m_tf_period;
        mtsFunctionRead m_crtk_interfaces_provided;
        void crtk_interfaces_provided_updated_handler(void);
    };

    std::map<std::pair<std::string, std::string>, factory *> m_factories;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mts_ros_crtk_bridge);

#endif // _mts_ros_crtk_bridge_h
