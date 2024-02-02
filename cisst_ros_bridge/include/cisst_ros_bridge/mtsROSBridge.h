/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

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
#if ROS1
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Empty.h>
#elif ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#endif

// conversion methods
#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>


// ----------------------------------------------------
// Publisher
// ----------------------------------------------------

class mtsROSPublisherBase
{
public:
    //! m_function used to pull data from the cisst component
    mtsFunctionRead m_function;

    mtsROSPublisherBase(const std::string & name,
                        cisst_ral::node_ptr_t node,
                        const bool latched):
        m_name(name),
        m_node(node),
        m_latched(latched)
    {}

    virtual ~mtsROSPublisherBase() {}

    virtual bool Execute(void) = 0;

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
    bool m_latched;
#if ROS1
    std::shared_ptr<ros::Publisher> m_publisher;
#endif

};

template <typename _cisst_t, typename _ros_t>
class mtsROSPublisher: public mtsROSPublisherBase
{
public:
    mtsROSPublisher(const std::string & name,
                    cisst_ral::node_ptr_t node,
                    const uint32_t queue_size = 5,
                    const bool latched = false):
        mtsROSPublisherBase(name, node, latched)
    {
        cisst_ral::create_publisher<_ros_t>(m_publisher, m_node, m_name, queue_size, m_latched);
        if (!m_publisher) {
            std::cerr << "No publisher" << std::endl;
        }
    }
    virtual ~mtsROSPublisher() {
        cisst_ral::publisher_shutdown(m_publisher);
    }
    bool Execute(void) {
        if ((cisst_ral::nb_subscribers(m_publisher) == 0) && !m_latched) {
            return true;
        }
        mtsExecutionResult result = m_function(m_cisst_data);
        if (result) {
            if (mts_cisst_to_ros::header(m_cisst_data, m_ros_data, m_node, m_name)) {
                mtsCISSTToROS(m_cisst_data, m_ros_data, m_name);
                m_publisher->publish(m_ros_data);
                return true;
            }
        } else {
            CISST_RAL_ERROR("mtsROSPublisher::Execute: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSPublisher::Execute: " << result
                              << " for " << m_name << std::endl;
        }
        return false;
    }

protected:
#if ROS2
    typedef typename rclcpp::Publisher<_ros_t> publisher_t;
    typename publisher_t::SharedPtr m_publisher;
#endif
    _cisst_t m_cisst_data;
    _ros_t m_ros_data;
};

class mtsROSEventVoidPublisher: public mtsROSPublisherBase
{
    typedef CISST_RAL_MSG(std_msgs, Empty) ros_t;
public:
    mtsROSEventVoidPublisher(const std::string & name,
                             cisst_ral::node_ptr_t node,
                             const uint32_t queue_size = 100,
                             const bool latched = true):
        mtsROSPublisherBase(name, node, latched)
    {
        cisst_ral::create_publisher<ros_t>(m_publisher, m_node, m_name, queue_size, m_latched);
    }
    virtual ~mtsROSEventVoidPublisher() {
        cisst_ral::publisher_shutdown(m_publisher);
    }
    bool Execute(void) {
        return true;
    }
    void EventHandler(void) {
        m_publisher->publish(m_ros_data);
    }
private:
#if ROS2
    typedef typename rclcpp::Publisher<ros_t> publisher_t;
    typename publisher_t::SharedPtr m_publisher;
#endif
    ros_t m_ros_data;
};


template <typename _cisst_t, typename _ros_t>
class mtsROSEventWritePublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventWritePublisher(const std::string & name,
                              cisst_ral::node_ptr_t node,
                              const uint32_t queue_size = 100,
                              const bool latched = true):
        mtsROSPublisherBase(name, node, latched)
    {
        cisst_ral::create_publisher<_ros_t>(m_publisher, m_node, m_name, queue_size, m_latched);
    }
    virtual ~mtsROSEventWritePublisher() {
        cisst_ral::publisher_shutdown(m_publisher);
    }
    bool Execute(void) {
        return true;
    }
    void EventHandler(const _cisst_t & cisst_data) {
        if ((cisst_ral::nb_subscribers(m_publisher) == 0) && !m_latched) {
            return;
        }
        if (mts_cisst_to_ros::header(cisst_data, m_ros_data, m_node, m_name)) {
            mtsCISSTToROS(cisst_data, m_ros_data, m_name);
            m_publisher->publish(m_ros_data);
        }
    }

protected:
#if ROS2
    typedef typename rclcpp::Publisher<_ros_t> publisher_t;
    typename publisher_t::SharedPtr m_publisher;
#endif
    _ros_t m_ros_data;
};


class mtsROStf2Broadcaster: public mtsROSPublisherBase
{
public:
    mtsROStf2Broadcaster(const std::string & name,
                         cisst_ral::node_ptr_t node):
        mtsROSPublisherBase(name, node, false),
#if ROS1
        m_broadcaster()
#elif ROS2
        m_broadcaster(node)
#endif
    {}
    virtual ~mtsROStf2Broadcaster() {
    }

    bool Execute(void) {
        mtsExecutionResult result = m_function(m_cisst_data);
        if (result) {
            // first check if it's new
            if (m_cisst_data.Timestamp() > m_last_timestamp) {
                m_last_timestamp = m_cisst_data.Timestamp();
                // then convert and check if the data is valid
                if (mts_cisst_to_ros::header(m_cisst_data, m_ros_data, m_node, m_name)) {
                    mtsCISSTToROS(m_cisst_data, m_ros_data, m_name);
                    m_broadcaster.sendTransform(m_ros_data);
                    return true;
                }
            }
        } else if (result.Value() != mtsExecutionResult::FUNCTION_NOT_BOUND) {
            CISST_RAL_ERROR("mtsROStf2Broadcaster::Execute: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROStf2Broadcaster::Execute: " << result
                              << " for " << m_name << std::endl;
            return false;
        }
        return false;
    }

protected:
    tf2_ros::TransformBroadcaster m_broadcaster;
    CISST_RAL_MSG(geometry_msgs, TransformStamped) m_ros_data;
    prmPositionCartesianGet m_cisst_data;
    double m_last_timestamp = 0.0;
};


class mtsROSEventWriteLog: public mtsROSPublisherBase
{
public:
    typedef enum {ROS_LOG_DEBUG, ROS_LOG_INFO, ROS_LOG_WARN, ROS_LOG_ERROR, ROS_LOG_FATAL} level_t;

    mtsROSEventWriteLog(const level_t level,
                        cisst_ral::node_ptr_t node):
        mtsROSPublisherBase("log", node, false),
        m_level(level)
    {}
    virtual ~mtsROSEventWriteLog() {}

    bool Execute(void) {
        return true;
    }

    void EventHandler(const mtsMessage & message) {
        switch (m_level) {
        case ROS_LOG_DEBUG:
            CISST_RAL_DEBUG("%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_INFO:
            CISST_RAL_INFO("%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_WARN:
            CISST_RAL_WARN("%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_ERROR:
            CISST_RAL_ERROR("%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_FATAL:
            CISST_RAL_FATAL("%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        default:
            break;
        }
    }
protected:
    const level_t m_level;
};


// ----------------------------------------------------
// Subscriber
// ----------------------------------------------------

template <typename _cisst_t, typename _ros_t>
class mtsROSSubscriberWrite
{
public:
    typedef mtsROSSubscriberWrite<_cisst_t, _ros_t> this_t;
    mtsROSSubscriberWrite(const std::string & name,
                          cisst_ral::node_ptr_t node):
        m_name(name),
        m_node(node)
    {
#if ROS1
        m_subscriber =
            std::make_shared<ros::Subscriber>(m_node->subscribe(m_name,
                                                                1,
                                                                &this_t::Callback,
                                                                this));
#elif ROS2
        m_subscriber =
            m_node->create_subscription<_ros_t>(m_name,
                                                1, std::bind(&this_t::Callback,
                                                             this,
                                                             std::placeholders::_1));
#endif
    }
    virtual ~mtsROSSubscriberWrite() {
        cisst_ral::subscriber_shutdown(m_subscriber);
    }
    void Callback(const _ros_t & ros_data) {
        mts_ros_to_cisst::header(ros_data, m_cisst_data, m_node);
        mtsROSToCISST(ros_data, m_cisst_data);
        mtsExecutionResult result = m_function(m_cisst_data);
        if (!result) {
            CISST_RAL_ERROR("mtsROSSubscriberWrite:Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSSubscriberWrite:Callback: " << result
                              << " for " << m_name << std::endl;
        }
    }

    mtsFunctionWrite m_function;

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
#if ROS1
    std::shared_ptr<ros::Subscriber> m_subscriber;
#elif ROS2
    typedef rclcpp::Subscription<_ros_t> subscriber_t;
    typename subscriber_t::SharedPtr m_subscriber;
 #endif
    _cisst_t m_cisst_data;
};


class mtsROSSubscriberVoid
{
public:
    typedef mtsROSSubscriberVoid this_t;
    typedef CISST_RAL_MSG(std_msgs, Empty) ros_t;
    mtsROSSubscriberVoid(const std::string & name,
                         cisst_ral::node_ptr_t node):
        m_name(name),
        m_node(node)
    {
#if ROS1
        m_subscriber =
            std::make_shared<ros::Subscriber>(m_node->subscribe(m_name,
                                                                1,
                                                                &mtsROSSubscriberVoid::Callback,
                                                                this));
#elif ROS2
        m_subscriber =
            m_node->create_subscription<ros_t>(m_name,
                                               1, std::bind(&this_t::Callback,
                                                            this,
                                                            std::placeholders::_1));
#endif
    }
    virtual ~mtsROSSubscriberVoid() {
        cisst_ral::subscriber_shutdown(m_subscriber);
    }
    void Callback(const ros_t & CMN_UNUSED(ros_data)) {
        mtsExecutionResult result = m_function();
        if (!result) {
            CISST_RAL_ERROR("mtsROSSubscriberVoid:Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSSubscriberVoid::Callback: " << result
                              << " for " << m_name  << std::endl;
        }
    }

    mtsFunctionVoid m_function;

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
#if ROS1
    std::shared_ptr<ros::Subscriber> m_subscriber;
#elif ROS2
    typedef rclcpp::Subscription<ros_t> subscriber_t;
    typename subscriber_t::SharedPtr m_subscriber;
 #endif
};


template <typename _cisst_t, typename _ros_t>
class mtsROSSubscriberStateTable
{
public:
    typedef mtsROSSubscriberStateTable<_cisst_t, _ros_t> this_t;
    mtsROSSubscriberStateTable(const std::string & name,
                               cisst_ral::node_ptr_t node,
                               const size_t & table_size):
        m_state_table(table_size, name),
        m_name(name),
        m_node(node)
    {
#if ROS1
        m_subscriber =
            std::make_shared<ros::Subscriber>(m_node->subscribe(m_name,
                                                                1,
                                                                &this_t::Callback,
                                                                this));
#elif ROS2
        m_subscriber =
            m_node->create_subscription<_ros_t>(m_name,
                                                1, std::bind(&this_t::Callback,
                                                             this,
                                                             std::placeholders::_1));
#endif
        m_state_table.AddData(m_cisst_data, m_name);
    }
    virtual ~mtsROSSubscriberStateTable() {
        cisst_ral::subscriber_shutdown(m_subscriber);
    }
    void Callback(const _ros_t & ros_data) {
        m_state_table.Start();
        mts_ros_to_cisst::header(ros_data, m_cisst_data, m_node);
        mtsROSToCISST(ros_data, m_cisst_data);
        m_state_table.Advance();
    }

    mtsStateTable m_state_table;
    _cisst_t m_cisst_data;

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
#if ROS1
    std::shared_ptr<ros::Subscriber> m_subscriber;
#elif ROS2
    typedef rclcpp::Subscription<_ros_t> subscriber_t;
    typename subscriber_t::SharedPtr m_subscriber;
#endif
};


template <typename _cisst_t, typename _ros_t>
class mtsROSCommandWritePublisher
{
public:
    mtsROSCommandWritePublisher(const std::string & name,
                                cisst_ral::node_ptr_t node,
                                const uint32_t queue_size = 100,
                                const bool latched = false):
        m_name(name),
        m_node(node),
        m_latched(latched)
    {
        cisst_ral::create_publisher<_ros_t>(m_publisher, m_node, m_name, queue_size, m_latched);
    }
    virtual ~mtsROSCommandWritePublisher() {
        cisst_ral::publisher_shutdown(m_publisher);
    }
    void Command(const _cisst_t & cisst_data) {
        if ((cisst_ral::nb_subscribers(m_publisher) == 0) && !m_latched) {
            return;
        }
        if (mts_cisst_to_ros::header(cisst_data, m_ros_data, m_node, m_name)) {
            mtsCISSTToROS(cisst_data, m_ros_data, m_name);
            m_publisher->publish(m_ros_data);
        }
    }

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
    bool m_latched;
#if ROS1
    std::shared_ptr<ros::Publisher> m_publisher;
#elif ROS2
    typedef typename rclcpp::Publisher<_ros_t> publisher_t;
    typename publisher_t::SharedPtr m_publisher;
#endif
    _ros_t m_ros_data;
};


class mtsROSCommandVoidPublisher
{
public:
    typedef CISST_RAL_MSG(std_msgs, Empty) ros_t;
    mtsROSCommandVoidPublisher(const std::string & name,
                               cisst_ral::node_ptr_t node,
                               const uint32_t queue_size = 100,
                               const bool latched = false):
        m_name(name),
        m_node(node),
        m_latched(latched)
    {
        cisst_ral::create_publisher<ros_t>(m_publisher, m_node, m_name, queue_size, m_latched);
    }
    virtual ~mtsROSCommandVoidPublisher() {
        cisst_ral::publisher_shutdown(m_publisher);
    }
    void Command(void)
    {
        m_publisher->publish(m_ros_data);
    }

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
    bool m_latched;
#if ROS1
    std::shared_ptr<ros::Publisher> m_publisher;
#elif ROS2
    typedef typename rclcpp::Publisher<ros_t> publisher_t;
    typename publisher_t::SharedPtr m_publisher;
#endif
    ros_t m_ros_data;
};


// ----------------------------------------------------
// Services
// ----------------------------------------------------
template <typename _cisst_response_t,
          typename _ros_service_t>
class mtsROSCommandReadService
{
public:
    typedef mtsROSCommandReadService<_cisst_response_t,
                                     _ros_service_t> this_t;
    mtsFunctionRead m_function;

    mtsROSCommandReadService(const std::string name,
                             cisst_ral::node_ptr_t node):
        m_name(name),
        m_node(node)
    {
#if ROS1
        m_service_server = m_node->advertiseService(m_name,
                                                   &this_t::Callback, this);
#elif ROS2
        m_service_server =
            node->create_service<_ros_service_t>(m_name,
                                                 std::bind(&this_t::Callback,
                                                           this,
                                                           std::placeholders::_1,
                                                           std::placeholders::_2));
#endif
    }
#if ROS1
    bool Callback(typename _ros_service_t::Request & CMN_UNUSED(request),
                  typename _ros_service_t::Response & response) {
        mtsExecutionResult result = m_function(m_cisst_response);
        if (result) {
            if (mts_cisst_to_ros::header(m_cisst_response, response, m_node, m_name)) {
                mtsCISSTToROS(m_cisst_response, response, m_name);
                return true;
            }
        } else {
            CISST_RAL_ERROR("mtsROSCommandReadService::Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSCommandReadService::Callback: " << result
                              << " for " << m_name << std::endl;
        }
        return false;
    }
#elif ROS2
    bool Callback(const std::shared_ptr<typename _ros_service_t::Request> CMN_UNUSED(request),
                  std::shared_ptr<typename _ros_service_t::Response> response) {
        mtsExecutionResult result = m_function(m_cisst_response);
        if (result) {
            if (mts_cisst_to_ros::header(m_cisst_response, *response, m_node, m_name)) {
                mtsCISSTToROS(m_cisst_response, *response, m_name);
                return true;
            }
        } else {
            CISST_RAL_ERROR("mtsROSCommandReadService::Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSCommandReadService::Callback: " << result
                              << " for " << m_name << std::endl;
        }
        return false;
    }
#endif

protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
    _cisst_response_t m_cisst_response;
#if ROS1
    ros::ServiceServer m_service_server;
#elif ROS2
    typedef rclcpp::Service<_ros_service_t> service_server_t;
    typename service_server_t::SharedPtr m_service_server;
#endif
};


template <typename _cisst_request_t,
          typename _cisst_response_t,
          typename _ros_service_t>
class mtsROSCommandQualifiedReadService
{
public:
    typedef mtsROSCommandQualifiedReadService<_cisst_request_t,
                                              _cisst_response_t,
                                              _ros_service_t> this_t;

    mtsFunctionQualifiedRead m_function;

    mtsROSCommandQualifiedReadService(const std::string name,
                                      cisst_ral::node_ptr_t node):
        m_name(name),
        m_node(node)
    {
#if ROS1
        m_service_server = m_node->advertiseService(m_name,
                                                   &this_t::Callback, this);
#elif ROS2
        m_service_server =
            node->create_service<_ros_service_t>(m_name,
                                                 std::bind(&this_t::Callback,
                                                           this,
                                                           std::placeholders::_1,
                                                           std::placeholders::_2));
#endif
    }
#if ROS1
    bool Callback(typename _ros_service_t::Request & ros_request,
                  typename _ros_service_t::Response & ros_response) {
        mts_ros_to_cisst::header(ros_request, m_cisst_request, m_node);
        mtsROSToCISST(ros_request, m_cisst_request);
        mtsExecutionResult result = m_function(m_cisst_request, m_cisst_response);
        if (result) {
            if (mts_cisst_to_ros::header(m_cisst_response, ros_response, m_node, m_name)) {
                mtsCISSTToROS(m_cisst_response, ros_response, m_name);
                return true;
            }
        } else {
            CISST_RAL_ERROR("mtsROSCommandQualifiedReadService::Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSCommandQualifiedReadService::Callback: " << result
                              << " for " << m_name << std::endl;
        }
        return false;
    }
#elif ROS2
    bool Callback(const std::shared_ptr<typename _ros_service_t::Request> request,
                  std::shared_ptr<typename _ros_service_t::Response> response) {
         mts_ros_to_cisst::header(*request, m_cisst_request, m_node);
        mtsROSToCISST(*request, m_cisst_request);
        mtsExecutionResult result = m_function(m_cisst_request, m_cisst_response);
        if (result) {
            if (mts_cisst_to_ros::header(m_cisst_response, *response, m_node, m_name)) {
                mtsCISSTToROS(m_cisst_response, *response, m_name);
                return true;
            }
        } else {
            CISST_RAL_ERROR("mtsROSCommandQualifedReadService::Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSCommandQualifiedReadService::Callback: " << result
                              << " for " << m_name << std::endl;
        }
        return false;
    }
#endif
    
protected:
    std::string m_name;
    cisst_ral::node_ptr_t m_node;
    _cisst_request_t m_cisst_request;
    _cisst_response_t m_cisst_response;
#if ROS1
    ros::ServiceServer m_service_server;
#elif ROS2
    typedef rclcpp::Service<_ros_service_t> service_server_t;
    typename service_server_t::SharedPtr m_service_server;
#endif
};


// ----------------------------------------------------
// Bridge
// ----------------------------------------------------

/*! Base component to convert cisst/SAW commands and event to/from ROS
  topics.  This component starts without any cisstMultiTask
  interface nor commands nor functions.  The user can configure this
  component by adding one of the following:

  - AddPublisherFromCommandRead
  [required interface][function read] -> [topic publish] (periodic)

  - AddPublisherFromEventVoid
  [required interface][event void] -> [topic publish]

  - AddPublisherFromEventWrite
  [required interface][event write] -> [topic publish]

  - Addtf2BroadcasterFromCommandRead
  [required interface][function read] -> [broadcast tf] (periodic)
  The function read must get prmPositionCartesianGet

  - AddSubscriberToCommandVoid
  [required interface][function void] <- [topic subscribe]

  - AddSubscriberToCommandWrite
  [required interface][function write] <- [topic subscribe]

  - AddPublisherFromCommandWrite
  [provided interface][command write] -> [topic publish]

  - AddPublisherFromCommandVoid
  [provided interface][command void] -> [topic publish]

  - AddSubscriberToCommandRead
  [provided interface][command read] <- [topic subscribe]

  - AddSubscriberToEventVoid
  [providedInterface][event void] <- [topic subscribe]

  - AddSubscriberToEventWrite
  [providedInterface][event write] <- [topic subscribe]

  For each of these methods (except the ones with Void commands or
  events), the user needs to provided the cisst/SAW and ROS types
  used to convert the data back and forth.  Since the conversion
  method has to be defined at compilation time, this is done using
  template specialization:

  mtsROSBridge bridge("publisher", 5.0 * cmn_ms);
  bridge.AddPublisherFromCommandRead
  <vctDoubleVec, cisst_msgs::DoubleVec>
  ("required",
  "GetValue1",
  "/sawROSExample/get_value_1");

  The first template parameter is the cisst type used for the
  command, the second parameter is the ROS type used to publish.  At
  compilation time, the compiler will look for one of the following
  overloaded method:
  - void mtsCISSTToROS(const _cisstType & in, _ros_t out, const std::string & debugInfo)
  - void mtsROSToCISST(const _ros_t & in, _cisstType out, const std::string & debugInfo)

  Some default conversion methods are provided in mtsROSToCISST.h
  and mtsCISSTToROS.h.

*/
class mtsROSBridge: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    /*!
      \brief Constructor

      \param name component name
      \param period_in_seconds thread period
      \param spin call spinOnce() in run() is set to true
      \param sig true to install default signal handler, if
      set to false, either install your own handler or
      rely on cisst cleanup()
    */
    CISST_DEPRECATED mtsROSBridge(const std::string & name,
                                  const double period_in_seconds,
                                  const bool spin = false,
                                  const bool sig = true,
                                  cisst_ral::node_ptr_t = nullptr);

    mtsROSBridge(const mtsTaskPeriodicConstructorArg & arg);

    /*!  Constructor using an existing ros::NodeHandle.  By default,
      spin is set to false (see also PerformsSpin) and this
      constructor doesn't redefine the signal handler.
    */
    mtsROSBridge(const std::string & name,
                 const double period_in_seconds,
                 cisst_ral::node_ptr_t node);

    ~mtsROSBridge();

    // taskPeriodic
    void Configure(const std::string & CMN_UNUSED(filename) = "");
    bool AddIntervalStatisticsInterface(const std::string & interface_name = "IntervalStatistics");
    void AddIntervalStatisticsPublisher(const std::string & ros_namespace,
                                        const std::string & name,
                                        const std::string & interface_name = "IntervalStatistics");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    /*! Request that the bridge calls ros::spinOnce in its Run method.
      This allows to piggy back on the existing thread/periodicity
      instead of using ros::spin in your main.  This can be used if
      you also have another library or toolkit that has its own event
      loop (i.e. Qt with QApplication.exec()).  If you use multiple
      mtsROSBridge, make sure there's only one bridge with spin turned
      on. */
    inline void PerformsSpin(const bool spin) {
        m_spin = spin;
    }

    // --------- Required interface

    // --------- Publisher ------------------

    /*! Add a read function to a cisstMultiTask required interface.
      When connected to an existing provided interface, this allows
      to read some data from an existing cisstMultiTask component
      and publish it to ROS.  This action is performed periodically,
      it is triggered by the periodicity defined when the bridge
      (this class) is constructed.

      \param interface_required Name of the required interface to be created
      \param function Name of the read function added to the interface
      \param name Name of the topic used to publish
    */
    template <typename _cisst_t, typename _ros_t>
    bool AddPublisherFromCommandRead(const std::string & interface_required,
                                     const std::string & function,
                                     const std::string & name,
                                     const uint32_t queue_size = 100,
                                     const bool latched = false);

    /*! Add an event handler (void) to a cisstMultiTask required
      interface.  When connected to an existing provided interface,
      this allows to handle events from an existing cisstMultiTask
      component and publish it to ROS using std_msgs::Empty.

      \param interface_required Name of the required interface to be created
      \param event Name of the event to handle
      \param name Name of the topic used to publish
    */
    bool AddPublisherFromEventVoid(const std::string & interface_required,
                                   const std::string & event,
                                   const std::string & name,
                                   const uint32_t queue_size = 100,
                                   const bool latched = true);

    /*! Add an event handler (write) to a cisstMultiTask required
      interface.  When connected to an existing provided interface,
      this allows to handle events from an existing cisstMultiTask
      component and publish it to ROS after converting from _cisst_t
      to _ros_t.

      \param interface_required Name of the required interface to be created
      \param event Name of the event to handle
      \param name Name of the topic used to publish
    */
    template <typename _cisst_t, typename _ros_t>
    bool AddPublisherFromEventWrite(const std::string & interface_required,
                                    const std::string & event,
                                    const std::string & name,
                                    const uint32_t queue_size = 100,
                                    const bool latched = true);

    // --------- Subscriber ------------------

    /*! Add a write function to a cisstMultiTask required interface.
      When connected to an existing provided interface, this allows
      to send commands to an existing cisstMultiTask component from
      a ROS subscriber after converting from _ros_t to _cisst_t.

      \param interface_required Name of the required interface to be created
      \param function Name of the write function
      \param name Name of the topic this subscribes to
    */
    template <typename _cisst_t, typename _ros_t>
    bool AddSubscriberToCommandWrite(const std::string & interface_required,
                                     const std::string & function,
                                     const std::string & name);

    /*! Add a void function to a cisstMultiTask required interface.
      When connected to an existing provided interface, this allows
      to send commands to an existing cisstMultiTask component from
      a ROS subscriber (receiving std_msgs::Empty).

      \param interface_required Name of the required interface to be created
      \param function Name of the void function
      \param name Name of the topic this subscribes to
    */
    bool AddSubscriberToCommandVoid(const std::string & interface_required,
                                    const std::string & function,
                                    const std::string & name);

    // -------- tf2 broadcasters
    bool Addtf2BroadcasterFromCommandRead(const std::string & interface_required,
                                          const std::string & function);

    // --------- Events to ROS log

    /*! Add an event handler (write) to a cisstMultiTask required
      interface.  When connected to an existing provided interface,
      this allows to handle events with a std::string payload from
      an existing cisstMultiTask component and log the message using
      either ROS_DEBUG, ROS_INFO, ROS_WARN, CISST_RAL_ERROR or ROS_FATAL
      based on the level selected (see
      mtsROSEventWriteLog::LogLevel).

      \param interface_required Name of the required interface to be created
      \param event Name of the event to handle
      \param level Level used to log in ROS
    */
    bool AddLogFromEventWrite(const std::string & interface_required,
                              const std::string & event,
                              const mtsROSEventWriteLog::level_t & level);

    // --------- Provided interface

    // --------- Publisher ------------------

    /*! Add a command (write) to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to execute write functions from an existing cisstMultiTask
      component and publish it to ROS after converting from _cisst_t
      to _ros_t.

      \param interface_provided Name of the provided interface to be created
      \param command Name of the write command added to the interface
      \param name Name of the topic used to publish
    */
    template <typename _cisst_t, typename _ros_t>
    bool AddPublisherFromCommandWrite(const std::string & interface_provided,
                                      const std::string & command,
                                      const std::string & name,
                                      const uint32_t queue_size = 100,
                                      const bool latched = false);

    /*! Add a command (void) to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to execute void functions from an existing cisstMultiTask
      component and publish it to ROS (using std_msgs::Empty).

      \param interface_provided Name of the provided interface to be created
      \param command Name of the void command added to the interface
      \param name Name of the topic used to publish
    */
    bool AddPublisherFromCommandVoid(const std::string & interface_provided,
                                     const std::string & command,
                                     const std::string & name,
                                     const uint32_t queue_size = 100,
                                     const bool latched = false);

    // --------- Subscriber ------------------

    /*! Add a command read to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to execute read functions from an existing cisstMultiTask
      component.  When the subscriber receives some data, it adds it
      to a local state table after converting from _ros_t to
      _cisst_t.  The data will be cached for the next call to the
      read command (see mtsStateTable).

      \param interface_provided Name of the provided interface to be created
      \param command Name of the void command added to the interface
      \param name Name of the topic this subscribes to
      \param table_size Size of the state table used to cache the data
    */
    template <typename _cisst_t, typename _ros_t>
    bool AddSubscriberToCommandRead(const std::string & interface_provided,
                                    const std::string & command,
                                    const std::string & name,
                                    const size_t & table_size = 500);

    /*! Add a void event to a cisstMultiTask provided interface.  When
      connected to an existing required interface, this allows to
      send void events to an existing cisstMultiTask component.
      When the subscriber receives some data (std_msgs::Empty), it
      triggers the event.

      \param interface_provided Name of the provided interface to be created
      \param event Name of the write event added to the interface
      \param name Name of the topic this subscribes to
    */
    bool AddSubscriberToEventVoid(const std::string & interface_provided,
                                  const std::string & event,
                                  const std::string & name);

    /*! Add a write event to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to send write events to an existing cisstMultiTask component.
      When the subscriber receives some data, it triggers the event
      after converting the data from _ros_t to _cisst_t.

      \param interface_provided Name of the provided interface to be created
      \param event Name of the write event added to the interface
      \param name Name of the topic this subscribes to
    */
    template <typename _cisst_t, typename _ros_t>
    bool AddSubscriberToEventWrite(const std::string & interface_provided,
                                   const std::string & event,
                                   const std::string & name);

    template <typename _cisst_response_t, typename _ros_service_t>
    bool AddServiceFromCommandRead(const std::string & interface_required,
                                   const std::string & function,
                                   const std::string & serviceName);
    template <typename _cisst_request_t, typename _cisst_response_t, typename _ros_service_t>
    bool AddServiceFromCommandQualifiedRead(const std::string & interface_required,
                                            const std::string & function,
                                            const std::string & serviceName);

protected:
    //! list of publishers
    typedef std::list<mtsROSPublisherBase *> publishers_t;
    publishers_t m_publishers;

    //! ros node
    cisst_ral::node_ptr_t m_node;

    //! spin flag, if set call spinOnce() in run
    bool m_spin;

    //! signal flag, if set use default signal handler from ros nodehandle
    bool m_signal;
};

template <typename _cisst_t, typename _ros_t>
bool mtsROSBridge::AddPublisherFromCommandRead(const std::string & interface_required,
                                               const std::string & function,
                                               const std::string & name,
                                               const uint32_t queue_size,
                                               const bool latched)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired
        = this->GetInterfaceRequired(interface_required);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interface_required);
    }
    if (!interfaceRequired) {
        CISST_RAL_ERROR("mtsROSBridge::AddPublisherFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromCommandRead: failed to create required interface \""
                                 << interface_required << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * new_pub =
        new mtsROSPublisher<_cisst_t, _ros_t>(name, m_node, queue_size, latched);
    if (!interfaceRequired->AddFunction(function, new_pub->m_function)) {
        CISST_RAL_ERROR("mtsROSBridge::AddPublisherFromCommandRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromCommandRead: failed to add function \""
                                 << function << "\" to interface required \""
                                 << interface_required << "\"" << std::endl;
        delete new_pub;
        return false;
    }
    m_publishers.push_back(new_pub);
    return true;
}


template <typename _cisst_t, typename _ros_t>
bool mtsROSBridge::AddSubscriberToCommandWrite(const std::string & interface_required,
                                               const std::string & function,
                                               const std::string & name)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interface_required);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interface_required);
    }
    if (!interfaceRequired) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToCommandWrite: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandWrite: failed to create required interface \""
                                 << interface_required << "\"" << std::endl;
        return false;
    }
    mtsROSSubscriberWrite<_cisst_t, _ros_t> * new_sub
        = new mtsROSSubscriberWrite<_cisst_t, _ros_t>(name, m_node);
    if (!interfaceRequired->AddFunction(function, new_sub->m_function)) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToCommandWrite: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandWrite: failed to add function \""
                                 << function << "\" to interface required \""
                                 << interface_required << "\"" << std::endl;
        delete new_sub;
        return false;
    }
    return true;
}


template <typename _cisst_t, typename _ros_t>
bool mtsROSBridge::AddPublisherFromEventWrite(const std::string & interface_required,
                                              const std::string & event,
                                              const std::string & name,
                                              const uint32_t queue_size,
                                              const bool latched)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interface_required);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interface_required);
    }

    mtsROSEventWritePublisher<_cisst_t, _ros_t> * new_pub
        = new mtsROSEventWritePublisher<_cisst_t, _ros_t>(name, m_node, queue_size, latched);
    if (!interfaceRequired->AddEventHandlerWrite(&mtsROSEventWritePublisher<_cisst_t, _ros_t>::EventHandler,
                                                 new_pub, event)) {
        CISST_RAL_ERROR("mtsROSBridge::AddPublisherFromEventWrite: failed to add event handler to required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromEventWrite: failed to add event handler for \""
                                 << event << "\" to required interface \""
                                 << interface_required << "\"" << std::endl;
        delete new_pub;
        return false;
    }
    m_publishers.push_back(new_pub);
    return true;
}


template <typename _cisst_t, typename _ros_t>
bool mtsROSBridge::AddPublisherFromCommandWrite(const std::string & interface_provided,
                                                const std::string & command,
                                                const std::string & name,
                                                const uint32_t queue_size,
                                                const bool latched)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interface_provided);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interface_provided);
    }

    mtsROSCommandWritePublisher<_cisst_t, _ros_t> * new_pub
        = new mtsROSCommandWritePublisher<_cisst_t, _ros_t>(name, m_node, queue_size, latched);
    if (!interfaceProvided->AddCommandWrite(&mtsROSCommandWritePublisher<_cisst_t, _ros_t>::Command,
                                            new_pub, command))
        {
            CISST_RAL_ERROR("mtsROSBridge::AddPublisherFromCommandWrite: failed to create provided interface.");
            CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromCommandWrite: failed to create provided interface \""
                                     << interface_provided << "\"" << std::endl;
            delete new_pub;
            return false;
        }
    return true;
}


template <typename _cisst_t, typename _ros_t>
bool mtsROSBridge::AddSubscriberToCommandRead(const std::string & interface_provided,
                                              const std::string & command,
                                              const std::string & name,
                                              const size_t & table_size)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interface_provided);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interface_provided);
    }

    mtsROSSubscriberStateTable<_cisst_t, _ros_t> * new_sub
        = new mtsROSSubscriberStateTable<_cisst_t, _ros_t>(name, m_node, table_size);
    if (!interfaceProvided->AddCommandReadState(new_sub->m_state_table,
                                                new_sub->m_cisst_data,
                                                command)) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToCommandRead: failed to add command read to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandRead: failed to add command read \""
                                 << command << "\" to provided interface \""
                                 << interface_provided << "\"" << std::endl;
        delete new_sub;
        return false;
    }
    return true;
}


template <typename _cisst_t, typename _ros_t>
bool mtsROSBridge::AddSubscriberToEventWrite(const std::string & interface_provided,
                                             const std::string & event,
                                             const std::string & name)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interface_provided);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interface_provided);
    }

    mtsROSSubscriberWrite<_cisst_t, _ros_t> * new_sub
        = new mtsROSSubscriberWrite<_cisst_t, _ros_t>(name, m_node);
    if (!interfaceProvided->AddEventWrite(new_sub->m_function,
                                          event, _cisst_t())) {
        CISST_RAL_ERROR("mtsROSBridge::AddSubscriberToEventWrite: failed to add event to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToEventWrite: failed to add event \""
                                 << event << "\" to provided interface \""
                                 << interface_provided << "\"" << std::endl;
        delete new_sub;
        return false;
    }
    return true;
}




template <typename _cisst_response_t, typename _ros_service_t>
bool mtsROSBridge::AddServiceFromCommandRead(const std::string & interface_required,
                                             const std::string & function,
                                             const std::string & serviceName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interface_required);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interface_required);
    }
    if (!interfaceRequired) {
        CISST_RAL_ERROR("mtsROSBridge::AddServiceFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandRead: failed to create required interface \""
                                 << interface_required << "\"" << std::endl;
        return false;
    }

    typedef mtsROSCommandReadService<_cisst_response_t, _ros_service_t> serviceType;
    serviceType * newService
        = new serviceType(serviceName, m_node);

    if (!interfaceRequired->AddFunction(function, newService->m_function)) {
        CISST_RAL_ERROR("mtsROSBridge::AddServiceFromCommandRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandRead: failed to create function \""
                                 << function << "\"" << std::endl;
        delete newService;
        return false;
    }
    return true;
}

template <typename _cisst_request_t, typename _cisst_response_t, typename _ros_service_t>
bool mtsROSBridge::AddServiceFromCommandQualifiedRead(const std::string & interface_required,
                                                      const std::string & function,
                                                      const std::string & serviceName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interface_required);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interface_required);
    }
    if (!interfaceRequired) {
        CISST_RAL_ERROR("mtsROSBridge::AddServiceFromCommandQualifiedRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandQualifiedRead: failed to create required interface \""
                                 << interface_required << "\"" << std::endl;
        return false;
    }

    typedef mtsROSCommandQualifiedReadService<_cisst_request_t, _cisst_response_t, _ros_service_t> serviceType;
    serviceType * newService
        = new serviceType(serviceName, m_node);

    if (!interfaceRequired->AddFunction(function, newService->m_function)) {
        CISST_RAL_ERROR("mtsROSBridge::AddServiceFromCommandQualifiedRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandQualifiedRead: failed to create function \""
                                 << function << "\"" << std::endl;
        delete newService;
        return false;
    }
    return true;
}

CMN_DECLARE_SERVICES_INSTANTIATION(mtsROSBridge);

#endif // _mtsROSBridge_h
