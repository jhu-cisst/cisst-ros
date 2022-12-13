/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/cisst_ros_crtk.h>
#include <ros/ros.h>

void cisst_ros_crtk::clean_namespace(std::string & _ros_namespace)
{
    _ros_namespace = ros::names::clean(_ros_namespace);
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), ' ', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), '-', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), '.', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), '(', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), ')', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), '[', '_');
    std::replace(_ros_namespace.begin(), _ros_namespace.end(), ']', '_');
}

void cisst_ros_crtk::get_crtk_command(const std::string & _full_command,
                                    std::string & _crtk_command)
{
    size_t pos = _full_command.find_last_of("/");
    if (pos == std::string::npos) {
        _crtk_command = _full_command;
    } else {
        _crtk_command = _full_command.substr(pos + 1);
    }
}

ros::NodeHandle * cisst_ros_crtk::ros_init(const std::string & name)
{
    // create fake argc/argv for ros::init
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0] = new char[name.size() + 1];
    strcpy(argv[0], name.c_str());
    int argc = 1;

    ros::init(argc, argv, name, ros::init_options::AnonymousName);
    return new ros::NodeHandle("");
}
