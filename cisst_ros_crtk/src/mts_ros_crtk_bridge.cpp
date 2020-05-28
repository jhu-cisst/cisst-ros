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

mts_ros_crtk_bridge::mts_ros_crtk_bridge(const std::string & _component_name,
                                         ros::NodeHandle * _node_handle):
    mtsTaskFromSignal(_component_name),
    m_node_handle_ptr(_node_handle)
{
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();

    // subscribers bridge is shared for all subscribers since we only want one
    m_subscribers_bridge =
        new mtsROSBridge(_component_name + "_subscribers", 0.1 * cmn_ms, m_node_handle_ptr);
    m_subscribers_bridge->AddIntervalStatisticsInterface();
    m_subscribers_bridge->PerformsSpin(true);
    _component_manager->AddComponent(m_subscribers_bridge);

    // event bridge
    m_events_bridge =
        new mtsROSBridge(_component_name + "_events", 0.1 * cmn_ms, m_node_handle_ptr);
    m_events_bridge->AddIntervalStatisticsInterface();
    _component_manager->AddComponent(m_events_bridge);

    // stats bridge
    m_stats_bridge =
        new mtsROSBridge(_component_name + "_stats", 200.0 * cmn_ms, m_node_handle_ptr);
    _component_manager->AddComponent(m_stats_bridge);
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

void mts_ros_crtk_bridge::bridge_interface_provided(const std::string & _component_name,
                                                    const std::string & _interface_name,
                                                    const std::string & _ros_namespace,
                                                    const double _publish_period_in_seconds,
                                                    const double _tf_period_in_seconds)
{
    // first make sure we can find the component to bridge
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    mtsComponent * _component = _component_manager->GetComponent(_component_name);
    if (!_component) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_provided: unable to find component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }
    // then try to find the interface
    mtsInterfaceProvided * _interface_provided = _component->GetInterfaceProvided(_interface_name);
    if (!_interface_provided) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_provided: unable to find provided interface \""
                                 << _interface_name << "\" on component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }

    // clean ROS namespace
    std::string _clean_namespace = _ros_namespace;
    clean_namespace(_clean_namespace);

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    // create new pub/tf bridges for this provided interface if needed
    mtsComponent * _existing_component;
    mtsROSBridge * _pub_bridge;
    mtsROSBridge * _tf_bridge;

    bool _pub_bridge_used = false;
    bool _tf_bridge_used = false;

    bool _pub_bridge_is_new;
    bool _tf_bridge_is_new;

    const std::string _pub_bridge_name =  this->GetName() + "_pub_" + _clean_namespace;
    _existing_component = _component_manager->GetComponent(_pub_bridge_name);
    if (_existing_component) {
        _pub_bridge_is_new = false;
        _pub_bridge = dynamic_cast<mtsROSBridge *>(_existing_component);
    } else {
        _pub_bridge_is_new = true;
        _pub_bridge = new mtsROSBridge(_pub_bridge_name,
                                       _publish_period_in_seconds, m_node_handle_ptr);
    }

    const std::string _tf_bridge_name =  this->GetName() + "_tf_" + _clean_namespace;
    _existing_component = _component_manager->GetComponent(_tf_bridge_name);
    if (_existing_component) {
        _tf_bridge_is_new = false;
        _tf_bridge = dynamic_cast<mtsROSBridge *>(_existing_component);
    } else {
        _tf_bridge_is_new = true;
        _tf_bridge = new mtsROSBridge(_tf_bridge_name,
                                      _tf_period_in_seconds, m_node_handle_ptr);
    }

    // add trailing / for clean namespace
    if (!_clean_namespace.empty()) {
        _clean_namespace.append("/");
    }

    std::string _crtk_command;
    std::string _ros_topic;

    // write commands
    for (auto & _command :  _interface_provided->GetNamesOfCommandsWrite()) {
        // get the CRTK command so we know which template type to use
        get_crtk_command(_command, _crtk_command);
        _ros_topic = _clean_namespace + _command;
        if ((_crtk_command == "servo_jp")
            || (_crtk_command == "move_jp")) {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
                (_required_interface_name, _command, _ros_topic);
        } else  if (_crtk_command == "servo_jf") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
                (_required_interface_name, _command, _ros_topic);
        } else if ((_crtk_command == "servo_cp")
                   || (_crtk_command == "move_cp")) {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::TransformStamped>
                (_required_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "servo_cf") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
                (_required_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "state_command") {
            m_subscribers_bridge->AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
                (_required_interface_name, _command, _ros_topic);
        }
    }

    // read commands
    for (auto & _command : _interface_provided->GetNamesOfCommandsRead()) {
        // get the CRTK command so we know which template type to use
        get_crtk_command(_command, _crtk_command);
        _ros_topic = _clean_namespace + _command;
        if ((_crtk_command == "measured_js")
            || (_crtk_command == "setpoint_js")) {
            _pub_bridge_used = true;
            _pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                (_interface_name, _command, _ros_topic);
        } else  if ((_crtk_command == "measured_cp")
                    || (_crtk_command == "setpoint_cp")) {
            _pub_bridge_used = true;
            _pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
                (_interface_name, _command, _ros_topic);
            // tf broadcast
            if (_crtk_command == "measured_cp") {
                _tf_bridge_used = true;
                _tf_bridge->Addtf2BroadcasterFromCommandRead(_interface_name, _command);
            }
        } else if (_crtk_command == "measured_cv") {
            _pub_bridge_used = true;
            _pub_bridge->AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "measured_cf") {
            _pub_bridge_used = true;
            _pub_bridge->AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "jacobian") {
            _pub_bridge_used = true;
            _pub_bridge->AddPublisherFromCommandRead<vctDoubleMat, std_msgs::Float64MultiArray>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "operating_state") {
            m_subscribers_bridge->AddServiceFromCommandRead<prmOperatingState, crtk_msgs::trigger_operating_state>
                (_required_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "period_statistics") {
            std::string _namespace = _component_name + "_" + _interface_name;
            std::transform(_namespace.begin(), _namespace.end(), _namespace.begin(), tolower);
            clean_namespace(_namespace);
            m_stats_bridge->AddIntervalStatisticsPublisher("stats/" + _namespace,
                                                           _component_name, _interface_name);
        }
    }

    // write events
    for (auto & _event : _interface_provided->GetNamesOfEventsWrite()) {
        // get the CRTK command so we know which template type to use
        get_crtk_command(_event, _crtk_command);
        _ros_topic = _clean_namespace + _event;
        if (_crtk_command == "operating_state") {
            m_events_bridge->AddPublisherFromEventWrite<prmOperatingState, crtk_msgs::operating_state>
                (_required_interface_name, _event, _ros_topic);
        } else if (_crtk_command == "error") {
            m_events_bridge->AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
                (_required_interface_name, _event, _ros_topic);
            m_events_bridge->AddLogFromEventWrite(_required_interface_name + "-ros-log", _event,
                                                  mtsROSEventWriteLog::ROS_LOG_ERROR);
        } else if (_crtk_command == "warning") {
            m_events_bridge->AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
                (_required_interface_name, _event, _ros_topic);
            m_events_bridge->AddLogFromEventWrite(_required_interface_name + "-ros-log", _event,
                                                  mtsROSEventWriteLog::ROS_LOG_WARN);
        } else if (_crtk_command == "status") {
            m_events_bridge->AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
                (_required_interface_name, _event, _ros_topic);
            m_events_bridge->AddLogFromEventWrite(_required_interface_name + "-ros-log", _event,
                                                  mtsROSEventWriteLog::ROS_LOG_INFO);
        }
    }

    // buttons are a pain, they tend to have one interface per button
    // with a single write event called "Button".  By convention, the
    // button interface is using the name of the device it is attached
    // too as prefix (e.g. ForceDimension Falcon00 has button
    // Falcon00-Left, SensablePhantom left has button leftButton1).
    // So we look at all interfaces on the component that match the
    // interface_name and have an event write called "Button".
    const size_t _prefix_size =_interface_name.size();
    for (auto & _button_interface : _component->GetNamesOfInterfacesProvided()) {
        // can only be a prefix if shorter
        if (_button_interface.size() > _prefix_size) {
            if (std::equal(_interface_name.begin(),
                           _interface_name.end(),
                           _button_interface.begin())) {
                // this interface qualifies based on the name, does it
                // contain an event write "Button"?
                auto _interface_candidate = _component->GetInterfaceProvided(_button_interface);
                auto _events = _interface_candidate->GetNamesOfEventsWrite();
                if (std::find(_events.begin(), _events.end(), "Button") != _events.end()) {
                    // remove heading - or _
                    size_t _offset = 0;
                    const char _first_char = _button_interface.at(_prefix_size);
                    if ((_first_char == '-') || (_first_char == '_')) {
                        _offset = 1;
                    }
                    std::string _button_name = _button_interface.substr(_prefix_size + _offset);
                    // put all to lower case to be more ROS alike
                    std::transform(_button_name.begin(), _button_name.end(), _button_name.begin(), tolower);
                    // add and connect interface to event bridge
                    m_events_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                        (_button_interface, "Button", _clean_namespace + _button_name);
                    m_connections.Add(m_events_bridge->GetName(), _button_interface,
                                      _component_name, _button_interface);
                }
            }
        }
    }

    // connect interfaces
    if (m_subscribers_bridge->GetInterfaceRequired(_required_interface_name)) {
        m_connections.Add(m_subscribers_bridge->GetName(), _required_interface_name,
                          _component_name, _interface_name);
    }
    if (m_events_bridge->GetInterfaceRequired(_required_interface_name)) {
        m_connections.Add(m_events_bridge->GetName(), _required_interface_name,
                          _component_name, _interface_name);
    }
    if (m_events_bridge->GetInterfaceRequired(_required_interface_name + "-ros-log")) {
        m_connections.Add(m_events_bridge->GetName(), _required_interface_name + "-ros-log",
                          _component_name, _interface_name);
    }

    if (_tf_bridge_used) {
        if (_tf_bridge_is_new) {
            _component_manager->AddComponent(_tf_bridge);
        }
        m_connections.Add(_tf_bridge->GetName(), _interface_name,
                          _component_name, _interface_name);
        if (_tf_bridge->AddIntervalStatisticsInterface()) {
            std::string _tf_namespace = "stats/tf_" + _component_name + "_" + _interface_name;
            clean_namespace(_tf_namespace);
            std::transform(_tf_namespace.begin(), _tf_namespace.end(), _tf_namespace.begin(), tolower);
            m_stats_bridge->AddIntervalStatisticsPublisher(_tf_namespace,
                                                           _tf_bridge->GetName());
        }
    } else {
        delete _tf_bridge;
    }

    if (_pub_bridge_used) {
        if (_pub_bridge_is_new) {
            _component_manager->AddComponent(_pub_bridge);
        }
        m_connections.Add(_pub_bridge->GetName(), _interface_name,
                          _component_name, _interface_name);
        if (_pub_bridge->AddIntervalStatisticsInterface()) {
            std::string _pub_namespace = "stats/publishers_" + _component_name + "_" + _interface_name;
            clean_namespace(_pub_namespace);
            std::transform(_pub_namespace.begin(), _pub_namespace.end(), _pub_namespace.begin(), tolower);
            m_stats_bridge->AddIntervalStatisticsPublisher(_pub_namespace,
                                                           _pub_bridge->GetName());
        }
    } else {
        delete _pub_bridge;
    }
}

void mts_ros_crtk_bridge::clean_namespace(std::string & _ros_namespace)
{
    _ros_namespace = ros::names::clean(_ros_namespace);
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), ' ', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), '-', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), '.', '_');
}


void mts_ros_crtk_bridge::get_crtk_command(const std::string & _full_command,
                                           std::string & _crtk_command)
{
    size_t pos = _full_command.find_last_of("/");
    if (pos == std::string::npos) {
        _crtk_command = _full_command;
    } else {
        _crtk_command = _full_command.substr(pos + 1);
    }
}
