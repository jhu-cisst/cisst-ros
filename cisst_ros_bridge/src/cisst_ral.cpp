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

#include <cisst_ros_bridge/cisst_ral.h>

#if ROS1

cisst_ral::ral::ral(int & argc, char * argv[], const std::string & node_name, bool anonymous_name)
{
    init(argc, argv, node_name, anonymous_name);
}

cisst_ral::ral::ral(const std::string & node_name, bool anonymous_name)
{
    // create fake argc/argv
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[node_name.size() + 1];
    strcpy(argv[0], node_name.c_str());
    int argc = 1;
    init(argc, argv, node_name, anonymous_name);
}

void cisst_ral::ral::init(int & argc, char * argv[], const std::string & node_name, bool anonymous_name)
{
    if (anonymous_name) {
        ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
    } else {
        ros::init(argc, argv, node_name);
    }
    for (int i = 0; i < argc; ++i) {
        m_stripped_arguments.push_back(argv[i]);
    }
    m_node = std::make_shared<ros::NodeHandle>();
}

cisst_ral::ral::~ral()
{
}

#elif ROS2

cisst_ral::ral::ral(int & argc, char * argv[], const std::string & node_name, bool anonymous_name)
{
    init(argc, argv, node_name, anonymous_name);
}

cisst_ral::ral::ral(const std::string & node_name, bool)
{
    if (!rclcpp::ok()) {
        // create fake argc/argv
        typedef char * char_pointer;
        char_pointer * argv = new char_pointer[1];
        argv[0]= new char[node_name.size() + 1];
        strcpy(argv[0], node_name.c_str());
        int argc = 1;
        rclcpp::init(argc, argv);
    }
    m_node = std::make_shared<rclcpp::Node>(node_name);
    m_stripped_arguments.push_back(node_name);
}

void cisst_ral::ral::init(int & argc, char * argv[], const std::string & node_name, bool)
{
    m_stripped_arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    // reconstruct argc/argv from stripped arguments
    typedef char * char_ptr;
    argc = m_stripped_arguments.size();
    argv = reinterpret_cast<char_ptr *>(malloc(argc * sizeof(char_ptr)));
    for (int i = 0; i < argc; ++i) {
        argv[i] = reinterpret_cast<char_ptr>(malloc(m_stripped_arguments.at(i).size() + 1));
        strcpy(argv[i], m_stripped_arguments.at(i).c_str());
    }
    m_node = std::make_shared<rclcpp::Node>(node_name);
}

cisst_ral::ral::~ral()
{
}

#endif
