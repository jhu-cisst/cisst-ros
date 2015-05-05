/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsROSBridge_h
#define _mtsROSBridge_h

// cisst include
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

// ros include
#include <ros/ros.h>
#include <std_msgs/Empty.h>

// conversion methods
#include "cisst_ros_bridge/mtsCISSTToROS.h"
#include "cisst_ros_bridge/mtsROSToCISST.h"


// ----------------------------------------------------
// Publisher
// ----------------------------------------------------

class mtsROSPublisherBase
{
public:
    //! Function used to pull data from the cisst component
    mtsFunctionRead Function;
    //! ROS publisher to publish the converted data
    ros::Publisher Publisher;

    virtual bool Execute(void) = 0;
};

template <typename _mtsType, typename _rosType>
class mtsROSPublisher: public mtsROSPublisherBase
{
public:
    mtsROSPublisher(const std::string & rosTopicName, ros::NodeHandle & node) {
        Publisher = node.advertise<_rosType>(rosTopicName, 5);
    }
    ~mtsROSPublisher() {
        //! \todo, how to remove the topic from the node?
    }

    bool Execute(void) {
        mtsExecutionResult result = Function(CISSTData);
        if (result) {
            mtsCISSTToROS(CISSTData, ROSData);
            Publisher.publish(ROSData);
            return true;
        }
        return false;
    }

protected:
    _mtsType CISSTData;
    _rosType ROSData;
};

class mtsROSEventVoidPublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventVoidPublisher(const std::string & rosTopicName, ros::NodeHandle & node)
    {
        Publisher = node.advertise<std_msgs::Empty>(rosTopicName, 5);
    }
    ~mtsROSEventVoidPublisher(){
        //! \todo remove the topic from the node
    }
    bool Execute(void){return true;}

    void EventHandler(){
        Publisher.publish(mEmptyMsg);
    }
private:
    std_msgs::Empty mEmptyMsg;
};


template <typename _mtsType, typename _rosType>
class mtsROSEventWritePublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventWritePublisher(const std::string & rosTopicName, ros::NodeHandle & node){
        Publisher = node.advertise<_rosType>(rosTopicName, 5);
    }
    ~mtsROSEventWritePublisher(){}

    bool Execute(void){return true;}

    void EventHandler(const _mtsType& CISSTData)
    {
        mtsCISSTToROS(CISSTData, ROSData);
        Publisher.publish(ROSData);
    }

protected:
    _rosType ROSData;
};


// ----------------------------------------------------
// Subscriber
// ----------------------------------------------------


template <typename _mtsType, typename _rosType>
class mtsROSSubscriberWrite
{
public:
    typedef mtsROSSubscriberWrite<_mtsType, _rosType> ThisType;
    mtsROSSubscriberWrite(const std::string & rosTopicName, ros::NodeHandle & node) {
        Subscriber = node.subscribe(rosTopicName, 1, &ThisType::Callback, this);
    }
    ~mtsROSSubscriberWrite() {
        // \todo, how to remove the subscriber from the node?
    }

    void Callback(const _rosType & rosData) {
        mtsROSToCISST(rosData, CISSTData);
        mtsExecutionResult result = Function(CISSTData);
        if (!result) {
            std::cerr << result << std::endl;
        }
    }

    mtsFunctionWrite Function;

protected:
    ros::Subscriber Subscriber;
    _mtsType CISSTData;
};


class mtsROSSubscriberVoid
{
public:
    mtsROSSubscriberVoid(const std::string & rosTopicName, ros::NodeHandle & node){
        Subscriber = node.subscribe(rosTopicName, 1, &mtsROSSubscriberVoid::Callback, this);
    }
    ~mtsROSSubscriberVoid(){}

    void Callback(const std_msgs::Empty & CMN_UNUSED(rosData)) {
        mtsExecutionResult result = Function();
        if (!result) {
            std::cerr << result << std::endl;
        }
    }

    mtsFunctionVoid Function;

protected:
    ros::Subscriber Subscriber;
};


template <typename _mtsType, typename _rosType>
class mtsROSSubscriberStateTable
{
public:
    typedef mtsROSSubscriberStateTable<_mtsType, _rosType> ThisType;
    mtsROSSubscriberStateTable(const std::string & rosTopicName,
                               ros::NodeHandle & node,
                               const size_t & tableSize):
        StateTable(tableSize, rosTopicName)
    {
        Subscriber = node.subscribe(rosTopicName, 1, &ThisType::Callback, this);
        StateTable.AddData(CISSTData, rosTopicName);
    }
    ~mtsROSSubscriberStateTable() {
        // \todo, how to remove the subscriber from the node?
    }

    void Callback(const _rosType & rosData) {
        StateTable.Start();
        mtsROSToCISST(rosData, CISSTData);
        StateTable.Advance();
    }

    mtsStateTable StateTable;
    _mtsType CISSTData;

protected:
    ros::Subscriber Subscriber;
};


template <typename _mtsType, typename _rosType>
class mtsROSCommandWritePublisher
{
public:
    mtsROSCommandWritePublisher(const std::string & rosTopicName, ros::NodeHandle & node) {
        Publisher = node.advertise<_rosType>(rosTopicName, 5);
    }
    ~mtsROSCommandWritePublisher() {}

    void Command(const _mtsType & CISSTData)
    {
        mtsCISSTToROS(CISSTData, ROSData);
        Publisher.publish(ROSData);
    }

protected:
    ros::Publisher Publisher;
    _rosType ROSData;
};


class mtsROSCommandVoidPublisher
{
public:
    mtsROSCommandVoidPublisher(const std::string & rosTopicName, ros::NodeHandle & node) {
        Publisher = node.advertise<std_msgs::Empty>(rosTopicName, 5);
    }
    ~mtsROSCommandVoidPublisher() {}

    void Command(void)
    {
        Publisher.publish(mEmptyMsg);
    }

protected:
    ros::Publisher Publisher;
    std_msgs::Empty mEmptyMsg;
};

// ----------------------------------------------------
// Bridge
// ----------------------------------------------------

class mtsROSBridge: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    /*!
     \brief Constructor

     \param componentName component name
     \param periodInSeconds thread period
     \param spin call spinOnce() in run() is set to true
     \param sig true to install default signal handler, if
            set to false, either install your own handler or
            rely on cisst cleanup()
    */
    mtsROSBridge(const std::string & componentName,
                 double periodInSeconds,
                 bool spin = false,
                 bool sig = true,
                 ros::NodeHandle* nh = NULL);
    inline ~mtsROSBridge() {}

    // taskPeriodic
    void Configure(const std::string & CMN_UNUSED(filename) = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    // --------- Required interface

    // --------- Publisher ------------------
    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromCommandRead(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    template <typename _mtsType, typename _rosType>
    bool CISST_DEPRECATED AddPublisherFromReadCommand(const std::string & interfaceRequiredName,
                                                      const std::string & functionName,
                                                      const std::string & topicName) {
        return AddPublisherFromCommandRead<_mtsType, _rosType>(interfaceRequiredName, functionName, topicName);
    }

    bool AddPublisherFromEventVoid(const std::string & interfaceRequiredName,
                                   const std::string & eventName,
                                   const std::string & topicName);

    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromEventWrite(const std::string & interfaceRequiredName,
                                    const std::string & eventName,
                                    const std::string & topicName);

    // --------- Subscriber ------------------
    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToCommandWrite(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    template <typename _mtsType, typename _rosType>
    bool CISST_DEPRECATED AddSubscriberToWriteCommand(const std::string & interfaceRequiredName,
                                                      const std::string & functionName,
                                                      const std::string & topicName) {
        return AddSubscriberToCommandWrite<_mtsType, _rosType>(interfaceRequiredName, functionName, topicName);
    }

    bool AddSubscriberToCommandVoid(const std::string & interfaceRequiredName,
                                    const std::string & functionName,
                                    const std::string & topicName);

    bool CISST_DEPRECATED AddSubscriberToVoidCommand(const std::string & interfaceRequiredName,
                                                     const std::string & functionName,
                                                     const std::string & topicName) {
        return AddSubscriberToCommandVoid(interfaceRequiredName, functionName, topicName);
    }

    // --------- Provided interface

    // --------- Publisher ------------------
    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromCommandWrite(const std::string & interfaceProvidedName,
                                      const std::string & commandName,
                                      const std::string & topicName);

    bool AddPublisherFromCommandVoid(const std::string & interfaceProvidedName,
                                     const std::string & commandName,
                                     const std::string & topicName);

    // --------- Subscriber ------------------
    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToCommandRead(const std::string & interfaceProvidedName,
                                    const std::string & commandName,
                                    const std::string & topicName,
                                    const size_t & tableSize = 500);

    bool AddSubscriberToEventVoid(const std::string & interfaceProvidedName,
                                  const std::string & eventName,
                                  const std::string & topicName);

    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToEventWrite(const std::string & interfaceProvidedName,
                                   const std::string & eventName,
                                   const std::string & topicName);

protected:
    //! list of publishers
    typedef std::list<mtsROSPublisherBase*> PublishersType;
    PublishersType Publishers;

    //! ros node
    ros::NodeHandle * Node;

    //! spin flag, if set call spinOnce() in run
    bool mSpin;

    //! signal flag, if set use default signal handler from ros nodehandle
    bool mSignal;
};

template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromCommandRead(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROSBridge::AddPublisherFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddPublisherFromCommandRead: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * newPublisher = new mtsROSPublisher<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newPublisher->Function)) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommand: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommand: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToCommandWrite(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROS::AddSubscribeToWriteCommand: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscribeToWriteCommand: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSSubscriberWrite<_mtsType, _rosType> * newSubscriber = new mtsROSSubscriberWrite<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newSubscriber->Function)) {
        ROS_ERROR("mtsROS::AddSubscriberToWriteCommand: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToWriteCommand: failed to create function \""
                                 << functionName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromEventWrite(const std::string & interfaceRequiredName,
                                              const std::string & eventName,
                                              const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventWritePublisher<_mtsType, _rosType>* newPublisher = new mtsROSEventWritePublisher<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddEventHandlerWrite(&mtsROSEventWritePublisher<_mtsType, _rosType>::EventHandler, newPublisher, eventName))
    {
        ROS_ERROR("mtsROS::mtsROSEventWritePublisher: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSEventWritePublisher: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromCommandWrite(const std::string & interfaceProvidedName,
                                                const std::string & commandName,
                                                const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSCommandWritePublisher<_mtsType, _rosType>* newPublisher = new mtsROSCommandWritePublisher<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceProvided->AddCommandWrite(&mtsROSCommandWritePublisher<_mtsType, _rosType>::Command,
                                            newPublisher, commandName))
    {
        ROS_ERROR("mtsROSBridge::AddPublisherFromCommandWrite: failed to create provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddPublisherFromCommandWrite: failed to create provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToEventWrite(const std::string & interfaceProvidedName,
                                             const std::string & eventName,
                                             const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSSubscriberWrite<_mtsType, _rosType> * newSubscriber = new mtsROSSubscriberWrite<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceProvided->AddEventWrite(newSubscriber->Function,
                                          eventName, _mtsType())) {
        ROS_ERROR("mtsROSBridge::AddSubscriberToEventWrite: failed to add event to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddSubscriberToEventWrite: failed to add event \""
                                 << eventName << "\" to provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToCommandRead(const std::string & interfaceProvidedName,
                                              const std::string & commandName,
                                              const std::string & topicName,
                                              const size_t & tableSize)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSSubscriberStateTable<_mtsType, _rosType> * newSubscriber = new mtsROSSubscriberStateTable<_mtsType, _rosType>(topicName, *(this->Node), tableSize);
    if (!interfaceProvided->AddCommandReadState(newSubscriber->StateTable,
                                                newSubscriber->CISSTData,
                                                commandName)) {
        ROS_ERROR("mtsROSBridge::AddSubscriberToCommandRead: failed to add command read to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSBridge::AddSubscriberToCommandRead: failed to add command read \""
                                 << commandName << "\" to provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}

CMN_DECLARE_SERVICES_INSTANTIATION(mtsROSBridge);

#endif // _mtsROSBridge_h
