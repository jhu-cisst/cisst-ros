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

// conversion methods
#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>
#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <cisst_ros_crtk/mts_ros_crtk_bridge_required.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mts_ros_crtk_bridge_required, mtsROSBridge, mtsTaskPeriodicConstructorArg);

mts_ros_crtk_bridge_required::mts_ros_crtk_bridge_required(const std::string & _component_name,
                                                           ros::NodeHandle * _node_handle,
                                                           const double _period_in_seconds):
    mtsROSBridge(_component_name, _period_in_seconds, _node_handle)
{
    init();
}

mts_ros_crtk_bridge_required::mts_ros_crtk_bridge_required(const mtsTaskPeriodicConstructorArg & arg):
    mtsROSBridge(arg.Name, arg.Period, cisst_ros_crtk::ros_init(arg.Name))
{
    init();
}

void mts_ros_crtk_bridge_required::init(void)
{
}

mts_ros_crtk_bridge_required::~mts_ros_crtk_bridge_required(void)
{
}

void mts_ros_crtk_bridge_required::Configure(const std::string & _json_file)
{
    std::ifstream _json_stream;
    _json_stream.open(_json_file.c_str());

    Json::Value _json_config;
    Json::Reader _json_reader;
    if (!_json_reader.parse(_json_stream, _json_config)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration\n"
                                 << _json_reader.getFormattedErrorMessages();
        return;
    }

    ConfigureJSON(_json_config);
}

void mts_ros_crtk_bridge_required::ConfigureJSON(const Json::Value & _json_config)
{
    Json::Value _json_value;
}

void mts_ros_crtk_bridge_required::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
}

void mts_ros_crtk_bridge_required::bridge_interface_required(const std::string & _component_name,
                                                             const std::string & _interface_name,
                                                             const std::string & _ros_namespace)
{
    // first make sure we can find the component to bridge
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    mtsComponent * _component = _component_manager->GetComponent(_component_name);
    if (!_component) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_required: unable to find component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }
    // then try to find the interface
    mtsInterfaceRequired * _interface_required = _component->GetInterfaceRequired(_interface_name);
    if (!_interface_required) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_interface_required: unable to find required interface \""
                                 << _interface_name << "\" on component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }

    // provided interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _provided_interface_name = _component_name + "_for_" + _interface_name;

    // populate the newly created provided interface
    populate_interface_provided(_provided_interface_name,
                                _ros_namespace,
                                _interface_required->GetNamesOfFunctionsVoid(),
                                _interface_required->GetNamesOfFunctionsWrite(),
                                _interface_required->GetNamesOfFunctionsRead(),
                                _interface_required->GetNamesOfEventHandlersWrite());
    m_connections.Add(this->GetName(), _provided_interface_name,
                      _component_name, _interface_name);
}

void mts_ros_crtk_bridge_required::populate_interface_provided(const std::string & _interface_name,
                                                               const std::string & _ros_namespace,
                                                               const std::vector<std::string> & _void_commands,
                                                               const std::vector<std::string> & _write_commands,
                                                               const std::vector<std::string> & _read_commands,
                                                               const std::vector<std::string> & _write_events)
{
    // clean ROS namespace
    std::string _clean_namespace = _ros_namespace;
    cisst_ros_crtk::clean_namespace(_clean_namespace);

    // add trailing / for clean namespace
    if (!_clean_namespace.empty()) {
        _clean_namespace.append("/");
    }

    std::string _crtk_command;
    std::string _ros_topic;

    // void commands, using event bridge for low latence
    for (auto & _command :  _void_commands) {
        // get the CRTK command so we know which template type to use
        cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
        _ros_topic = _clean_namespace + _command;
        if ("hold") {
            this->AddPublisherFromCommandVoid(_interface_name, _command, _ros_topic);
        }
    }

    // write commands, using event bridge for low latence
    for (auto & _command :  _write_commands) {
        // get the CRTK command so we know which template type to use
        cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
        _ros_topic = _clean_namespace + _command;
        if ((_crtk_command == "servo_jp")
            || (_crtk_command == "servo_jr")
            || (_crtk_command == "move_jp")
            || (_crtk_command == "move_jr")) {
            this->AddPublisherFromCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
                (_interface_name, _command, _ros_topic);
        } else  if (_crtk_command == "servo_jv") {
            this->AddPublisherFromCommandWrite<prmVelocityJointSet, sensor_msgs::JointState>
                (_interface_name, _command, _ros_topic);
        } else  if (_crtk_command == "servo_jf") {
            this->AddPublisherFromCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
                (_interface_name, _command, _ros_topic);
        } else if ((_crtk_command == "servo_cp")
                   || (_crtk_command == "servo_cr")
                   || (_crtk_command == "move_cp")
                   || (_crtk_command == "move_cr")) {
            this->AddPublisherFromCommandWrite<prmPositionCartesianSet, geometry_msgs::PoseStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "servo_cf") {
            this->AddPublisherFromCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "state_command") {
            this->AddPublisherFromCommandWrite<std::string, crtk_msgs::StringStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "use_gravity_compensation") {
            this->AddPublisherFromCommandWrite<bool, std_msgs::Bool>
                (_interface_name, _command, _ros_topic);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "populate_interface_provided: write command \"" << _command
                                       << "\" is not recognized as a CRTK command so it was not automatically added to the provided interface \""
                                       << _interface_name << "\" " << std::endl;
        }
    }

    // read commands
    for (auto & _command : _read_commands) {
        // get the CRTK command so we know which template type to use
        cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
        _ros_topic = _clean_namespace + _command;
        if ((_crtk_command == "measured_js")
            || (_crtk_command == "setpoint_js")) {
            this->AddSubscriberToCommandRead<prmStateJoint, sensor_msgs::JointState>
                (_interface_name, _command, _ros_topic);
        } else  if ((_crtk_command == "measured_cp")
                    || (_crtk_command == "setpoint_cp")) {
            this->AddSubscriberToCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "measured_cv") {
            this->AddSubscriberToCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "measured_cf") {
            this->AddSubscriberToCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "jacobian") {
            this->AddSubscriberToCommandRead<vctDoubleMat, std_msgs::Float64MultiArray>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "operating_state") {
            this->AddSubscriberToCommandRead<prmOperatingState, crtk_msgs::operating_state>
                (_interface_name, _command, _ros_topic);
        } else if (_crtk_command == "period_statistics") {
            this->AddSubscriberToCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
                (_interface_name, _command, _ros_topic);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "populate_interface_provided: read command \"" << _command
                                       << "\" is not recognized as a CRTK command so it was not automatically added to the provided interface \""
                                       << _interface_name << "\" " << std::endl;
        }
    }

    // write events
    for (auto & _event : _write_events) {
        // get the CRTK command so we know which template type to use
        cisst_ros_crtk::get_crtk_command(_event, _crtk_command);
        _ros_topic = _clean_namespace + _event;
        if (_crtk_command == "input_data") {
            this->AddSubscriberToEventWrite<prmInputData, sensor_msgs::Joy>
                (_interface_name, _event, _ros_topic);
        } else if (_crtk_command == "Button") {
            this->AddSubscriberToEventWrite<prmEventButton, sensor_msgs::Joy>
                (_interface_name, _event, _clean_namespace); // for buttons, we just use the interface name
        } else if (_crtk_command == "operating_state") {
            this->AddSubscriberToEventWrite<prmOperatingState, crtk_msgs::operating_state>
                (_interface_name, _event, _ros_topic);
        } else if (_crtk_command == "error") {
            this->AddSubscriberToEventWrite<mtsMessage, std_msgs::String>
                (_interface_name, _event, _ros_topic);
        } else if (_crtk_command == "warning") {
            this->AddSubscriberToEventWrite<mtsMessage, std_msgs::String>
                (_interface_name, _event, _ros_topic);
        } else if (_crtk_command == "status") {
            this->AddSubscriberToEventWrite<mtsMessage, std_msgs::String>
                (_interface_name, _event, _ros_topic);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "populate_interface_provided: write event \"" << _event
                                       << "\" is not recognized as a CRTK command so it was not automatically added to the provided interface \""
                                       << _interface_name << "\" " << std::endl;
        }
    }
}
