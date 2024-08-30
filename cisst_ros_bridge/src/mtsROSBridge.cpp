/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <signal.h>   // ROS only supports Linux
#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES(mtsROSBridge);


mtsROSBridge::mtsROSBridge(const std::string & name,
                           const double period_in_seconds,
                           const bool spin,
                           const bool signal,
                           cisst_ral::node_ptr_t node):
    mtsTaskPeriodic(name, period_in_seconds),
    m_spin(spin),
    m_signal(signal)
{
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[strlen(name.c_str()) + 1];
    strcpy(argv[0], name.c_str());
    int argc = 1;

    if (node != nullptr) {
        m_node = node;
    } else {
#if ROS1
        if (m_signal) {
            ros::init(argc, argv, name);
        } else {
            ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
        }
        m_node = std::make_shared<ros::NodeHandle>();
#elif ROS2
        if (m_signal) {
            rclcpp::init(argc, argv);
        } else {
            rclcpp::init(argc, argv);
            rclcpp::uninstall_signal_handlers();
        }
        m_node = std::make_shared<rclcpp::Node>(name);
#endif
    }
}

mtsROSBridge::mtsROSBridge(const mtsTaskPeriodicConstructorArg &arg):
    mtsTaskPeriodic(arg),
    m_spin(false),
    m_signal(true)
{
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[strlen(arg.Name.c_str()) + 1];
    strcpy(argv[0], arg.Name.c_str());
    int argc = 1;

#if ROS1
    ros::init(argc, argv, arg.Name);
    m_node = std::make_shared<ros::NodeHandle>();
#elif ROS2
    rclcpp::init(argc, argv);
    m_node = std::make_shared<rclcpp::Node>(arg.Name);
#endif
}

mtsROSBridge::mtsROSBridge(const std::string & name,
                           const double period_in_seconds,
                           cisst_ral::node_ptr_t node):
    mtsTaskPeriodic(name, period_in_seconds),
    m_spin(false),
    m_signal(false)
{
    m_node = node;
}

void mtsROSBridge::Configure(const std::string & CMN_UNUSED(filename))
{
}

mtsROSBridge::~mtsROSBridge()
{
    for (auto pub : m_publishers) {
        delete(pub);
    }
    m_publishers.clear();
}

bool mtsROSBridge::AddIntervalStatisticsInterface(const std::string & interfaceName)
{
    // check if the interface already exists
    if (GetInterfaceProvided(interfaceName)) {
        return false;
    }
    // create an interface to get access to this component interval statistics
    mtsInterfaceProvided * controlInterface = AddInterfaceProvided(interfaceName);
    controlInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                          "period_statistics");
    return true;
}

void mtsROSBridge::AddIntervalStatisticsPublisher(const std::string & ros_namespace,
                                                  const std::string & componentName,
                                                  const std::string & interfaceName)
{
    // create an publisher to publish this component interval statistics
    std::string topicName = ros_namespace + "/period_statistics";
    this->AddPublisherFromCommandRead<mtsIntervalStatistics, CISST_RAL_MSG(cisst_msgs, IntervalStatistics)>
        (componentName + interfaceName, "period_statistics", topicName);

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(this->GetName(), componentName + interfaceName,
                              componentName, interfaceName);
}

void mtsROSBridge::Startup(void)
{
}

void mtsROSBridge::Cleanup(void)
{
    if (!m_signal) {
#if ROS1
        ros::requestShutdown();
#elif ROS2
        rclcpp::shutdown();
#endif
    }
}

void mtsROSBridge::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    for (auto pub : m_publishers) {
        pub->Execute();
    }

    if (m_spin) {
#if ROS1
        ros::spinOnce();
#elif ROS2
        rclcpp::spin_some(m_node);
#endif 
    }
}

bool mtsROSBridge::AddPublisherFromEventVoid(const std::string & interfaceRequiredName,
                                             const std::string & eventName,
                                             const std::string & topicName,
                                             const uint32_t queueSize,
                                             const bool latch)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventVoidPublisher* new_pub =
        new mtsROSEventVoidPublisher(topicName, m_node, queueSize, latch);
    if (!interfaceRequired->AddEventHandlerVoid(&mtsROSEventVoidPublisher::EventHandler, new_pub, eventName))
        {
            CISST_RAL_ERROR("mtsROSBridge::AddPublisherFromEventVoid: failed to add event handler to required interface.");
            CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromEventVoid: failed to add event handler for \""
                                     << eventName << "\" to required interface \""
                                     << interfaceRequiredName << "\"" << std::endl;
            delete new_pub;
            return false;
        }
    m_publishers.push_back(new_pub);
    return true;
}


bool mtsROSBridge::Addtf2BroadcasterFromCommandRead(const std::string & interfaceRequiredName,
                                                    const std::string & functionName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        CISST_RAL_ERROR("mtsROSBridge::Addtf2BroadcasterFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "Addtf2BroadcasterFromCommandRead: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * new_pub =
        new mtsROStf2Broadcaster(interfaceRequiredName + "::" + functionName, m_node);
    if (!interfaceRequired->AddFunction(functionName, new_pub->m_function)) {
        CISST_RAL_ERROR("mtsROSBridge::Addtf2BroadcasterFromCommandRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "Addtf2BroadcasterFromCommandRead: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete new_pub;
        return false;
    }
    m_publishers.push_back(new_pub);
    return true;
}


bool mtsROSBridge::AddLogFromEventWrite(const std::string & interfaceRequiredName,
                                        const std::string & eventName,
                                        const mtsROSEventWriteLog::level_t & level)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventWriteLog * new_pub = new mtsROSEventWriteLog(level, m_node);
    if (!interfaceRequired->AddEventHandlerWrite(&mtsROSEventWriteLog::EventHandler, new_pub, eventName)) {
        CISST_RAL_ERROR("mtsROSBridge::AddLogFromEventWrite: failed to add event handler to required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddLogFromEventWrite: failed to add event handler for \""
                                 << eventName << "\" to required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete new_pub;
        return false;
    }
    m_publishers.push_back(new_pub);
    return true;
}

bool mtsROSBridge::AddSubscriberToCommandVoid(const std::string & interfaceRequiredName,
                                              const std::string & functionName,
                                              const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToCommandVoid: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandVoid: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }

    mtsROSSubscriberVoid * new_sub = new mtsROSSubscriberVoid(topicName, m_node);
    if (!interfaceRequired->AddFunction(functionName, new_sub->m_function)) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToCommandVoid: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandVoid: failed to create function \""
                                 << functionName << "\"" << std::endl;
        delete new_sub;
        return false;
    }
    return true;
}

bool mtsROSBridge::AddPublisherFromCommandVoid(const std::string & interfaceProvidedName,
                                               const std::string & commandName,
                                               const std::string & topicName,
                                               const uint32_t queueSize,
                                               const bool latch)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSCommandVoidPublisher* new_pub =
        new mtsROSCommandVoidPublisher(topicName, m_node, queueSize, latch);
    if (!interfaceProvided->AddCommandVoid(&mtsROSCommandVoidPublisher::Command,
                                           new_pub, commandName)) {
        CISST_RAL_ERROR("mtsROSBridge::AddPublisherFromCommandVoid: failed to create provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddPublisherFromCommandVoid: failed to create provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete new_pub;
        return false;
    }
    return true;
}

bool mtsROSBridge::AddSubscriberToEventVoid(const std::string & interfaceProvidedName,
                                            const std::string & eventName,
                                            const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSSubscriberVoid * new_sub = new mtsROSSubscriberVoid(topicName, m_node);
    if (!interfaceProvided->AddEventVoid(new_sub->m_function,
                                         eventName)) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToEventVoid: failed to add event to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddSubscriberToEventVoid: failed to add event \""
                                 << eventName << "\" to provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete new_sub;
        return false;
    }
    return true;
}
