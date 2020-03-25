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

#include <cisst_ros_crtk/mts_ros_crtk_bridge.h>

// conversion methods
#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>
#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES(mts_ros_crtk_bridge);

mts_ros_crtk_bridge::mts_ros_crtk_bridge(const std::string & component_name,
                                         ros::NodeHandle * node_handle):
    mtsTaskFromSignal(component_name),
    m_node_handle_ptr(node_handle)
{
    mtsManagerLocal * component_manager = mtsComponentManager::GetInstance();

    // subscribers bridge is shared for all subscribers since we only want one
    m_subscribers_bridge =
        new mtsROSBridge(component_name + "_subscribers", 0.1 * cmn_ms, m_node_handle_ptr);
    m_subscribers_bridge->AddIntervalStatisticsInterface();
    m_subscribers_bridge->PerformsSpin(true);
    component_manager->AddComponent(m_subscribers_bridge);

    // event bridge
    m_events_bridge =
        new mtsROSBridge(component_name + "_events", 0.1 * cmn_ms, m_node_handle_ptr);
    m_events_bridge->AddIntervalStatisticsInterface();
    component_manager->AddComponent(m_events_bridge);

    // stats bridge
    m_stats_bridge =
        new mtsROSBridge(component_name + "_stats", 200.0 * cmn_ms, m_node_handle_ptr);
    component_manager->AddComponent(m_stats_bridge);
    m_stats_bridge->AddIntervalStatisticsPublisher("stats/subscribers", m_subscribers_bridge->GetName());
    m_stats_bridge->AddIntervalStatisticsPublisher("stats/events", m_events_bridge->GetName());
}

mts_ros_crtk_bridge::~mts_ros_crtk_bridge(void)
{

}

void mts_ros_crtk_bridge::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
}

void mts_ros_crtk_bridge::bridge_interface_provided(const std::string & component_name,
                                                    const std::string & interface_name,
                                                    const double publish_period_in_seconds,
                                                    const std::string & ros_namespace)
{
    // first make sure we can find the component to bridge
    mtsManagerLocal * component_manager = mtsComponentManager::GetInstance();
    mtsComponent * component = component_manager->GetComponent(component_name);
    if (!component) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_provided: unable to find component \""
                                 << component_name << "\"" << std::endl;
        return;
    }
    // then try to find the interface
    mtsInterfaceProvided * interface_provided = component->GetInterfaceProvided(interface_name);
    if (!component) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_provided: unable to find provided interface \""
                                 << interface_name << "\" on component \""
                                 << component_name << "\"" << std::endl;
        return;
    }

    // clean ROS namespace
    std::string clean_namespace = ros_namespace;
    std::replace(clean_namespace.begin(), clean_namespace.end(), ' ', '_');
    std::replace(clean_namespace.begin(), clean_namespace.end(), '-', '_');
    std::replace(clean_namespace.begin(), clean_namespace.end(), '.', '_');

    // create a new bridge for this provided interface
    mtsROSBridge * pub_bridge =
        new mtsROSBridge(this->GetName() + "_" + clean_namespace,
                         publish_period_in_seconds, m_node_handle_ptr);

    // add trailing / for clean namespace
    clean_namespace.append("/");

    typedef std::vector<std::string> list_type;
    typedef list_type::const_iterator iter_type;
    list_type commands;
    iter_type iter, end;

    bool has_stats = false;

    // write commands
    commands = interface_provided->GetNamesOfCommandsWrite();
    end = commands.end();
    for (iter = commands.begin();
         iter != end;
         ++iter) {
        if (*iter == "servo_cp") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::TransformStamped>
                (interface_name, "servo_cp", clean_namespace + "servo_cp");
        } else if (*iter == "servo_cf") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
                (interface_name, "servo_cf", clean_namespace + "servo_cf");
        } else if (*iter == "move_cp") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::TransformStamped>
                (interface_name, "move_cp", clean_namespace + "move_cp");
        } else if (*iter == "state_command") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
                (interface_name, "state_command", clean_namespace + "state_command");
        }
    }

    // read commands
    commands = interface_provided->GetNamesOfCommandsRead();
    end = commands.end();
    for (iter = commands.begin();
         iter != end;
         ++iter) {
        if (*iter == "measured_js") {
            pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                (interface_name, "measured_js", clean_namespace + "measured_js");
        } else  if (*iter == "measured_cp") {
            pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
                (interface_name, "measured_cp", clean_namespace + "measured_cp");
        } else if (*iter == "measured_cv") {
            pub_bridge->AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
                (interface_name, "measured_cv", clean_namespace + "measured_cv");
        } else if (*iter == "measured_cf") {
            pub_bridge->AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
                (interface_name, "measured_cf", clean_namespace + "measured_cf");
        } else if (*iter == "gripper_measured_js") {
            pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                (interface_name, "gripper_measured_js", clean_namespace + "gripper/measured_js");
        } else if (*iter == "setpoint_jp") {
            pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                (interface_name, "setpoint_jp", clean_namespace + "setpoint_jp");
        } else if (*iter == "setpoint_cp") {
            pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
                (interface_name, "setpoint_cp", clean_namespace + "setpoint_cp");
        } else if (*iter == "operating_state") {
            m_subscribers_bridge->AddServiceFromCommandRead<prmOperatingState, crtk_msgs::trigger_operating_state>
                (interface_name, "operating_state", clean_namespace + "operating_state");
        } else if (*iter == "GetPeriodStatistics") {
            has_stats = true;
        }
    }

    // write events
    commands = interface_provided->GetNamesOfEventsWrite();
    end = commands.end();
    for (iter = commands.begin();
         iter != end;
         ++iter) {
        if (*iter == "operating_state") {
            m_events_bridge->AddPublisherFromEventWrite<prmOperatingState, crtk_msgs::operating_state>
                (interface_name, "operating_state", clean_namespace + "operating_state");
        } else if (*iter == "Error") {
            m_events_bridge->AddLogFromEventWrite(interface_name, "Error",
                                                  mtsROSEventWriteLog::ROS_LOG_ERROR);
        } else if (*iter == "Warning") {
            m_events_bridge->AddLogFromEventWrite(interface_name, "Warning",
                                                  mtsROSEventWriteLog::ROS_LOG_WARN);
        } else if (*iter == "Status") {
            m_events_bridge->AddLogFromEventWrite(interface_name, "Status",
                                                  mtsROSEventWriteLog::ROS_LOG_INFO);
        }
    }

    // connect interfaces
    if (m_subscribers_bridge->GetInterfaceRequired(interface_name)) {
            component_manager->Connect(m_subscribers_bridge->GetName(), interface_name,
                                       component_name, interface_name);
    }
    if (m_events_bridge->GetInterfaceRequired(interface_name)) {
        component_manager->Connect(m_events_bridge->GetName(), interface_name,
                                   component_name, interface_name);
    }

    // add stats for publisher itself
    pub_bridge->AddIntervalStatisticsInterface();
    component_manager->AddComponent(pub_bridge);
    component_manager->Connect(pub_bridge->GetName(), interface_name,
                               component_name, interface_name);
    m_stats_bridge->AddIntervalStatisticsPublisher("stats/publishers_" + interface_name,
                                                   pub_bridge->GetName());

    // stats from the component we're bridging
    if (has_stats) {
        m_stats_bridge->AddIntervalStatisticsPublisher("stats/" + interface_name,
                                                       component_name, interface_name);
    }
}

#if 0

        // Buttons
        NamesType buttons;
        forceDimension->GetButtonNames(name, buttons);
        const NamesType::iterator endButtons = buttons.end();
        NamesType::iterator button;
        for (button = buttons.begin();
             button != endButtons;
             ++button) {
            // sawForceDimension button names are device-button, use device/button for ROS
            std::string rosButton = *button;
            std::replace(rosButton.begin(), rosButton.end(), '-', '/');
            std::replace(rosButton.begin(), rosButton.end(), '.', '_');
            spin_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                (*button, "Button", rosButton);
            component_manager->Connect(spin_bridge->GetName(), *button,
                                       forceDimension->GetName(), *button);
        }
#endif
