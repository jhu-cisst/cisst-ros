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

#include <cisstMultiTask/mtsManagerComponentServices.h>

// conversion methods
#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>
#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mts_ros_crtk_bridge_provided, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mts_ros_crtk_bridge_provided::mts_ros_crtk_bridge_provided(const std::string & _component_name,
                                                           ros::NodeHandle * _node_handle,
                                                           const double _period_in_seconds):
    mtsTaskPeriodic(_component_name, _period_in_seconds),
    m_node_handle_ptr(_node_handle)
{
    init();
}

mts_ros_crtk_bridge_provided::mts_ros_crtk_bridge_provided(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    m_node_handle_ptr = cisst_ros_crtk::ros_init(arg.Name);
    init();
}

void mts_ros_crtk_bridge_provided::init(void)
{
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    const std::string _component_name = this->GetName();

    // component manager interface
    mtsInterfaceRequired * required = EnableDynamicComponentManagement();
    if (required) {
        required->SetMailBoxAndArgumentQueuesSize(1000);
        ManagerComponentServices->AddConnectionEventHandler(&mts_ros_crtk_bridge_provided::add_connection_event_handler, this);
    }

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

mts_ros_crtk_bridge_provided::~mts_ros_crtk_bridge_provided(void)
{
    if (m_subscribers_bridge) {
        delete m_subscribers_bridge;
    }
    if (m_events_bridge) {
        delete m_events_bridge;
    }
    if (m_stats_bridge) {
        delete m_stats_bridge;
    }
}

void mts_ros_crtk_bridge_provided::Configure(const std::string & _json_file)
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

void mts_ros_crtk_bridge_provided::ConfigureJSON(const Json::Value & _json_config)
{
    Json::Value _json_value;
    const Json::Value _interfaces = _json_config["interfaces"];
    if (_interfaces.empty()) {
        CMN_LOG_CLASS_INIT_WARNING << "ConfigureJSON: no \"interfaces\" defined, this will not add any ROS CRTK bridge"
                                   <<  std::endl;
    }
    for (unsigned int index = 0; index < _interfaces.size(); ++index) {
        _json_value = _interfaces[index]["component"];
        if (_json_value.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureJSON: all \"interfaces\" must define \"component\"" << std::endl;
            return;
        }
        // future implementation could also support interface-required
        std::string _component_name = _json_value.asString();
        _json_value = _interfaces[index]["interface-provided"];
        if (_json_value.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureJSON: all \"interfaces\" must define \"interface-provided\"" << std::endl;
            return;
        }
        std::string _interface_name = _json_value.asString();
        _json_value = _interfaces[index]["namespace"];
        if (_json_value.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureJSON: all \"interfaces\" must define \"namespace\"" << std::endl;
            return;
        }
        std::string _name = _json_value.asString();

        // optional fields
        double _publish_period = cisst_ros_crtk::bridge_provided_default_publish_period;
        _json_value = _interfaces[index]["publish-period"];
        if (!_json_value.empty()) {
            _publish_period = _json_value.asFloat();
        }
        double _tf_period = cisst_ros_crtk::bridge_provided_default_tf_period;
        _json_value = _interfaces[index]["tf-period"];
        if (!_json_value.empty()) {
            _tf_period = _json_value.asFloat();
        }

        // if set, only bridge CRTK commands that matches user provided list
        const Json::Value _bridge_only = _interfaces[index]["bridge-only"];
        for (unsigned int bo = 0; bo < _bridge_only.size(); ++bo) {
            m_bridge_only.insert(_bridge_only[bo].asString());
        }

        // and now add the bridge
        bridge_interface_provided(_component_name, _interface_name, _name,
                                  _publish_period, _tf_period);
    }

    // skip connecting interfaces in case users want to add more
    // commands/functions/events to bridge before connecting
    _json_value = _json_config["skip-connect"];
    if (!_json_value.empty() && _json_value.asBool()) {
        CMN_LOG_CLASS_INIT_WARNING << "ConfigureJSON: \"skip-connect\" set to \"true\", user is responsible for connecting components" <<  std::endl;
    } else {
        mtsComponentManager::GetInstance()->AddComponent(this);
        Connect();
    }
}

void mts_ros_crtk_bridge_provided::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
}

void mts_ros_crtk_bridge_provided::bridge_all_interfaces_provided(const std::string & _component_name,
                                                                  const std::string & _ros_namespace,
                                                                  const double _publish_period_in_seconds,
                                                                  const double _tf_period_in_seconds)
{
    // first make sure we can find the component to bridge
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    mtsComponent * _component = _component_manager->GetComponent(_component_name);
    if (!_component) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_all_interfaces_provided: unable to find component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }
    // search for all interfaces provided
    std::vector<std::string> _interfaces = _component->GetNamesOfInterfacesProvided();
    for (const auto & _interface : _interfaces) {
        auto _ros_sub_namespace = _interface;
        cisst_ros_crtk::clean_namespace(_ros_sub_namespace);
        bridge_interface_provided(_component_name,
                                  _interface,
                                  _ros_namespace + '/' + _ros_sub_namespace,
                                  _publish_period_in_seconds,
                                  _tf_period_in_seconds);
    }
}

void mts_ros_crtk_bridge_provided::bridge_interface_provided(const std::string & _component_name,
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
    cisst_ros_crtk::clean_namespace(_clean_namespace);

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

    // void commands
    for (auto & _command :  _interface_provided->GetNamesOfCommandsVoid()) {
        if (should_be_bridged(_command)) {
            // get the CRTK command so we know which template type to use
            cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
            _ros_topic = _clean_namespace + _command;
            if (_crtk_command == "hold") {
                m_subscribers_bridge->AddSubscriberToCommandVoid(_required_interface_name, _command, _ros_topic);
            }
        }
    }

    // write commands
    for (auto & _command :  _interface_provided->GetNamesOfCommandsWrite()) {
        if (should_be_bridged(_command)) {
            // get the CRTK command so we know which template type to use
            cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
            _ros_topic = _clean_namespace + _command;
            if ((_crtk_command == "servo_jp")
                || (_crtk_command == "servo_jr")
                || (_crtk_command == "move_jp")
                || (_crtk_command == "move_jr")) {
                m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
                    (_required_interface_name, _command, _ros_topic);
            } else  if (_crtk_command == "servo_jf") {
                m_subscribers_bridge->AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
                    (_required_interface_name, _command, _ros_topic);
            } else if ((_crtk_command == "servo_cp")
                       || (_crtk_command == "servo_cr")
                       || (_crtk_command == "move_cp")
                       || (_crtk_command == "move_cr")) {
                m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::PoseStamped>
                    (_required_interface_name, _command, _ros_topic);
            } else if (_crtk_command == "servo_cf") {
                m_subscribers_bridge->AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
                    (_required_interface_name, _command, _ros_topic);
            } else if (_crtk_command == "state_command") {
                m_subscribers_bridge->AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
                    (_required_interface_name, _command, _ros_topic);
            } else if (_crtk_command == "use_gravity_compensation") {
                m_subscribers_bridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
                    (_required_interface_name, _command, _ros_topic);
            } else if (_crtk_command == "servo_ci") {
                m_subscribers_bridge->AddSubscriberToCommandWrite<prmCartesianImpedanceGains, cisst_msgs::prmCartesianImpedanceGains>
                    (_required_interface_name, _command, _ros_topic);
            }
        }
    }

    // read commands
    for (auto & _command : _interface_provided->GetNamesOfCommandsRead()) {
        if (should_be_bridged(_command)) {
            // get the CRTK command so we know which template type to use
            cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
            _ros_topic = _clean_namespace + _command;
            if ((_crtk_command == "measured_js")
                || (_crtk_command == "setpoint_js")) {
                _pub_bridge_used = true;
                _pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
                    (_interface_name, _command, _ros_topic);
            } else  if ((_crtk_command == "measured_cp")
                        || (_crtk_command == "setpoint_cp")) {
                _pub_bridge_used = true;
                _pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
                    (_interface_name, _command, _ros_topic);
                // tf broadcast if not local/ since local will have
                // the same child id and tf2 will complain about
                // duplicates - even though they have different
                // reference frames
                if ((_crtk_command == "measured_cp")
                    && (_command.find("local/measured_cp") == std::string::npos)) {
                    _tf_bridge_used = true;
                    _tf_bridge->Addtf2BroadcasterFromCommandRead(_interface_name, _command);
                }
            } else  if ((_crtk_command == "measured_cp_array")
                        || (_crtk_command == "setpoint_cp_array")) {
                _pub_bridge_used = true;
                _pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianArrayGet, geometry_msgs::PoseArray>
                    (_interface_name, _command, _ros_topic);
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
                cisst_ros_crtk::clean_namespace(_namespace);
                m_stats_bridge->AddIntervalStatisticsPublisher("stats/" + _namespace,
                                                               _component_name, _interface_name);
            }
        }
    }

    // qualified read commands
    for (auto & _command : _interface_provided->GetNamesOfCommandsQualifiedRead()) {
        if (should_be_bridged(_command)) {
            // get the CRTK command so we know which template type to use
            cisst_ros_crtk::get_crtk_command(_command, _crtk_command);
            _ros_topic = _clean_namespace + _command;
            if (_crtk_command == "query_cp") {
                m_subscribers_bridge->AddServiceFromCommandQualifiedRead<vctDoubleVec, vctFrm4x4,
                                                                         cisst_msgs::QueryForwardKinematics>
                    (_required_interface_name, _command, _ros_topic);
            }
        }
    }

    // write events
    for (auto & _event : _interface_provided->GetNamesOfEventsWrite()) {
        if (should_be_bridged(_event)) {
            // get the CRTK command so we know which template type to use
            cisst_ros_crtk::get_crtk_command(_event, _crtk_command);
            _ros_topic = _clean_namespace + _event;
            if (_crtk_command == "input_data") {
                m_events_bridge->AddPublisherFromEventWrite<prmInputData, sensor_msgs::Joy>
                    (_required_interface_name, _event, _ros_topic);
            } else if (_crtk_command == "Button") {
                m_events_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                    (_required_interface_name, _event, _clean_namespace); // for buttons, we just use the interface name
            } else if (_crtk_command == "operating_state") {
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
            m_new_components.push_back(_tf_bridge->GetName());
        }
        m_connections.Add(_tf_bridge->GetName(), _interface_name,
                          _component_name, _interface_name);
        if (_tf_bridge->AddIntervalStatisticsInterface()) {
            std::string _tf_namespace = "stats/tf_" + _component_name + "_" + _interface_name;
            cisst_ros_crtk::clean_namespace(_tf_namespace);
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
            m_new_components.push_back(_pub_bridge->GetName());
        }
        m_connections.Add(_pub_bridge->GetName(), _interface_name,
                          _component_name, _interface_name);
        if (_pub_bridge->AddIntervalStatisticsInterface()) {
            std::string _pub_namespace = "stats/publishers_" + _component_name + "_" + _interface_name;
            cisst_ros_crtk::clean_namespace(_pub_namespace);
            std::transform(_pub_namespace.begin(), _pub_namespace.end(), _pub_namespace.begin(), tolower);
            m_stats_bridge->AddIntervalStatisticsPublisher(_pub_namespace,
                                                           _pub_bridge->GetName());
        }
    } else {
        delete _pub_bridge;
    }
}

void mts_ros_crtk_bridge_provided::add_factory_source(const std::string & _component_name,
                                                      const std::string & _interface_name,
                                                      const double _publish_period_in_seconds,
                                                      const double _tf_period_in_seconds)
{
    decltype(m_factories)::key_type source(_component_name, _interface_name);

    // first, make sure this doesn't already exist
    auto found = m_factories.find(source);
    if (found != m_factories.end()) {
        CMN_LOG_CLASS_INIT_WARNING << "add_factory_source: factory already added for component \""
                                   << _component_name << "\" with interface \""
                                   << _interface_name << "\"" << std::endl;
        return;
    }

    // save default periods for this interface
    factory * _new_factory = new factory;
    _new_factory->m_bridge = this;
    _new_factory->m_publish_period = _publish_period_in_seconds;
    _new_factory->m_tf_period = _tf_period_in_seconds;
    m_factories[source] = _new_factory;

    // create a new interface for the factory
    std::string _req_interface_name = "_" + _component_name + "_using_" + _interface_name + "_factory";
    mtsInterfaceRequired * _interface = AddInterfaceRequired(_req_interface_name);
    if (_interface) {
        _interface->AddFunction("crtk_interfaces_provided", _new_factory->m_crtk_interfaces_provided);
        _interface->AddEventHandlerVoid(&factory::crtk_interfaces_provided_updated_handler,
                                        _new_factory, "crtk_interfaces_provided_updated");
        // add this interface to required connections
        m_connections.Add(this->GetName(), _req_interface_name,
                          _component_name, _interface_name);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_factory_source: there's already an interface name \""
                                 << _req_interface_name << "\"" << std::endl;
        return;
    }
}

void mts_ros_crtk_bridge_provided::add_connection_event_handler(const mtsDescriptionConnection & _connection)
{
    auto _component_manager = mtsComponentManager::GetInstance();
    bool _all_connected = true;
    std::list<std::string> _started;
    for (const auto & _component_name : m_new_components) {
        auto _component = _component_manager->GetComponent(_component_name);
        _all_connected = _all_connected && _component->AreAllInterfacesRequiredConnected();
        if (_all_connected) {
            auto _component = _component_manager->GetComponent(_component_name);
            _component->Create();
            _component->Start();
            _started.push_back(_component_name);
        }
    }
    for (const auto & _component_name : _started) {
        m_new_components.erase(std::find(m_new_components.begin(),
                                         m_new_components.end(),
                                         _component_name));
    }
}

bool mts_ros_crtk_bridge_provided::should_be_bridged(const std::string & _command)
{
    if (m_bridge_only.empty()) {
        return true;
    }
    if (m_bridge_only.find(_command) != m_bridge_only.end()) {
        return true;
    }
    return false;
}

void mts_ros_crtk_bridge_provided::factory::crtk_interfaces_provided_updated_handler(void)
{
    // get all CRTK interfaces
    std::vector<mtsDescriptionInterfaceFullName> _sources;
    m_crtk_interfaces_provided(_sources);
    for (const auto & _source : _sources) {
        std::string _namespace = _source.ComponentName + "/" + _source.InterfaceName;
        cisst_ros_crtk::clean_namespace(_namespace);
        m_bridge->bridge_interface_provided(_source.ComponentName,
                                            _source.InterfaceName,
                                            _namespace,
                                            m_publish_period,
                                            m_tf_period);
        m_bridge->Connect();
        // after connect, we rely on the new connection event handler to create and start components
    }
}
