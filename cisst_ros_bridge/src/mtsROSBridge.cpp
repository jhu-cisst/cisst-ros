/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <signal.h>   // ROS only supports Linux
#include "cisst_ros_bridge/mtsROSBridge.h"

CMN_IMPLEMENT_SERVICES(mtsROSBridge);


mtsROSBridge::mtsROSBridge(const std::string & componentName,
                           const double periodInSeconds,
                           const bool spin,
                           const bool sig,
                           ros::NodeHandle * nh):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mSpin(spin),
    mSignal(sig)
{
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[strlen("mtsROSBridge") + 1];
    strcpy(argv[0], "mtsROSBridge");
    int argc = 1;

    if (nh != NULL) {
        Node = nh;
    } else {
        if (mSignal) {
            ros::init(argc, argv, componentName);
        } else {
            ros::init(argc, argv, componentName, ros::init_options::NoSigintHandler);
        }
        Node = new ros::NodeHandle;
    }
}

void mtsROSBridge::Configure(const std::string & CMN_UNUSED(filename))
{
}

void mtsROSBridge::AddIntervalStatisticsInterface(const std::string & interfaceName)
{
    // create an interface to get access to this component interval statistics
    mtsInterfaceProvided * controlInterface = AddInterfaceProvided(interfaceName);
    controlInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                          "GetPeriodStatistics");
}

void mtsROSBridge::AddIntervalStatisticsPublisher(const std::string & rosNamespace,
                                                  const std::string & componentName,
                                                  const std::string & interfaceName)
{
    // create an publisher to publish this component interval statistics
    std::string topicName = rosNamespace + "/period_statistics";
    this->AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        (componentName + interfaceName, "GetPeriodStatistics",
         topicName);

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(this->GetName(), componentName + interfaceName,
                              componentName, interfaceName);
}

void mtsROSBridge::Startup(void)
{
}

void mtsROSBridge::Cleanup(void)
{
    if (!mSignal) {
        ros::requestShutdown();
    }
}

void mtsROSBridge::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    const PublishersType::iterator end = Publishers.end();
    PublishersType::iterator iter;
    for (iter = Publishers.begin();
         iter != end;
         ++iter) {
        (*iter)->Execute();
    }

    if (mSpin) ros::spinOnce();
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

    mtsROSEventVoidPublisher* newPublisher =
        new mtsROSEventVoidPublisher(topicName, *(this->Node), queueSize, latch);
    if (!interfaceRequired->AddEventHandlerVoid(&mtsROSEventVoidPublisher::EventHandler, newPublisher, eventName))
    {
        ROS_ERROR("mtsROSBridge::AddPublisherFromEventVoid: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromEventVoid: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
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
        ROS_ERROR("mtsROSBridge::Addtf2BroadcasterFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "Addtf2BroadcasterFromCommandRead: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * newPublisher =
        new mtsROStf2Broadcaster(interfaceRequiredName + "::" + functionName);
    if (!interfaceRequired->AddFunction(functionName, newPublisher->Function)) {
        ROS_ERROR("mtsROSBridge::Addtf2BroadcasterFromCommandRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "Addtf2BroadcasterFromCommandRead: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


bool mtsROSBridge::AddLogFromEventWrite(const std::string & interfaceRequiredName,
                                        const std::string & eventName,
                                        const mtsROSEventWriteLog::LogLevel & level)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventWriteLog * newPublisher = new mtsROSEventWriteLog(level);
    if (!interfaceRequired->AddEventHandlerWrite(&mtsROSEventWriteLog::EventHandler, newPublisher, eventName))
    {
        ROS_ERROR("mtsROSBridge::AddLogFromEventWrite: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddLogFromEventWrite: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
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
        ROS_ERROR("mtsROSBridge::AddSubscriberToCommandVoid: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandVoid: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }

    mtsROSSubscriberVoid * newSubscriber = new mtsROSSubscriberVoid(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newSubscriber->Function)) {
        ROS_ERROR("mtsROSBridge::AddSubscriberToCommandVoid: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandVoid: failed to create function \""
                                 << functionName << "\"" << std::endl;
        delete newSubscriber;
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

    mtsROSCommandVoidPublisher* newPublisher =
        new mtsROSCommandVoidPublisher(topicName, *(this->Node), queueSize, latch);
    if (!interfaceProvided->AddCommandVoid(&mtsROSCommandVoidPublisher::Command,
                                           newPublisher, commandName))
    {
        ROS_ERROR("mtsROSBridge::AddPublisherFromCommandVoid: failed to create provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddPublisherFromCommandVoid: failed to create provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newPublisher;
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

    mtsROSSubscriberVoid * newSubscriber = new mtsROSSubscriberVoid(topicName, *(this->Node));
    if (!interfaceProvided->AddEventVoid(newSubscriber->Function,
                                          eventName)) {
        ROS_ERROR("mtsROSBridge::AddSubscriberToEventVoid: failed to add event to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddSubscriberToEventVoid: failed to add event \""
                                 << eventName << "\" to provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}
