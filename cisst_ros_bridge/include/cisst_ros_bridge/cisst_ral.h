/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2024-01-30

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _cisst_ral_h
#define _cisst_ral_h

#include <cisstCommon/cmnPortability.h>
#include <cisstCommon/cmnStrings.h>

// ros includes
#if ROS1

#include <ros/ros.h>

#define CISST_RAL_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define CISST_RAL_INFO(...)  ROS_INFO(__VA_ARGS__)
#define CISST_RAL_WARN(...)  ROS_WARN(__VA_ARGS__)
#define CISST_RAL_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define CISST_RAL_FATAL(...) ROS_FATAL(__VA_ARGS__)

#define CISST_RAL_MSG(package, message) package::message
#define CISST_RAL_SRV(package, service) package::service
#define CISST_RAL_SRV_REQ(package, service) package::service::Request
#define CISST_RAL_SRV_RES(package, service) package::service::Response

namespace cisst_ral {
    typedef std::shared_ptr<ros::NodeHandle> node_ptr_t;

    inline bool time_is_zero(const ros::Time & time) {
        return time.isZero();
    }

    inline double age_in_seconds(const ros::Time & time,
                                 node_ptr_t CMN_UNUSED(node)) {
        return (ros::Time::now() - time).toSec();
    }

    inline ros::Time now(node_ptr_t CMN_UNUSED(node)) {
        return ros::Time::now();
    }

    inline ros::Duration duration_from_seconds(const double & duration) {
        return ros::Duration(duration);
    }

    inline void spin(node_ptr_t) {
        ros::spin();
    }

    inline void shutdown(void) {
        ros::shutdown();
    }

    template <typename _ros_t>
    void create_publisher(std::shared_ptr<ros::Publisher> & publisher,
                          node_ptr_t node,
                          const std::string & topic,
                          const size_t queue_size,
                          const bool latched) {
        publisher = std::make_shared<ros::Publisher>(node->advertise<_ros_t>(topic, queue_size, latched));
        if (!publisher) {
            std::cerr << "not created" << std::endl;
        }
    }

    template <typename _pub_t>
    inline size_t nb_subscribers(_pub_t publisher) {
        return publisher->getNumSubscribers();
    }

    template <typename _pub_t>
    inline void publisher_shutdown(_pub_t publisher) {
        publisher->shutdown();
    }

    template <typename _pub_t>
    inline void subscriber_shutdown(_pub_t subscriber) {
        subscriber->shutdown();
    }
}

#elif ROS2

#include <rclcpp/rclcpp.hpp>

#define CISST_RAL_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define CISST_RAL_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define CISST_RAL_WARN(...)  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define CISST_RAL_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define CISST_RAL_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), __VA_ARGS__)

#define CISST_RAL_MSG(package, message) package::msg::message
#define CISST_RAL_SRV(package, service) package::srv::service
#define CISST_RAL_SRV_REQ(package, service) package::srv::service::Request
#define CISST_RAL_SRV_RES(package, service) package::srv::service::Response

namespace cisst_ral {
    typedef std::shared_ptr<rclcpp::Node> node_ptr_t;

    inline bool time_is_zero(const rclcpp::Time & time) {
        return ((time.seconds() == 0)
                && (time.nanoseconds() == 0));
    }

    inline double age_in_seconds(const rclcpp::Time & time,
                                 node_ptr_t node) {
        return (node->get_clock()->now() - time).seconds();
    }

    inline rclcpp::Time now(node_ptr_t node) {
        return node->get_clock()->now();
    }

    inline rclcpp::Duration duration_from_seconds(const double & duration) {
        return rclcpp::Duration::from_seconds(duration);
    }

    inline void spin(node_ptr_t node) {
        rclcpp::spin(node);
    }

    inline void shutdown(void) {
        rclcpp::shutdown();
    }

    template <typename _ros_t>
    void create_publisher(typename rclcpp::Publisher<_ros_t>::SharedPtr & publisher,
                          node_ptr_t node,
                          const std::string & topic,
                          const size_t queue_size,
                          const bool latched) {
        rclcpp::QoS qos(queue_size);
        if (latched) {
            qos.transient_local();
        }
        publisher =
            node->create_publisher<_ros_t>(topic, qos);
    }

    template <typename _pub_t>
    inline size_t nb_subscribers(_pub_t publisher) {
        return publisher->get_subscription_count();
    }

    template <typename _pub_t>
    inline void publisher_shutdown(_pub_t) {
        // publisher->shutdown();
    }

    template <typename _pub_t>
    inline void subscriber_shutdown(_pub_t) {
        // subscriber->shutdown();
    }
}

#endif

namespace cisst_ral {

    inline void clean_namespace(std::string & _ros_namespace) {
#if ROS1
        _ros_namespace = ros::names::clean(_ros_namespace);
#endif
        cmnStringReplaceAll(_ros_namespace, "//", "/");
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ' ', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '-', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '.', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '(', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ')', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '[', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ']', '_');
    }

    class ral
    {
    public:
        ral(int argc, char * argv[], const std::string & node_name, bool anonymous_name = true);
        ral(const std::string & node_name, bool anonymous_name = true);
        ~ral();

        inline node_ptr_t node(void) {
            return m_node;
        }

        typedef std::vector<std::string> stripped_arguments_t;
        inline const stripped_arguments_t & stripped_arguments(void) const {
            return m_stripped_arguments;
        }

    protected:
        void init(int argc,  char * argv[], const std::string & node_name, bool anonymous_name);
        std::string m_node_name;
        node_ptr_t m_node;
        stripped_arguments_t m_stripped_arguments;
    };
}

#endif // _cisst_ral_h
