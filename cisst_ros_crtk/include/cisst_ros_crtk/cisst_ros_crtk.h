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

#ifndef _cisst_ros_crtk_h
#define _cisst_ros_crtk_h

#include <string>
#include <cisstCommon/cmnUnits.h>

namespace ros {
    class NodeHandle;
}

namespace cisst_ros_crtk
{
    const double bridge_provided_default_publish_period = 10.0 * cmn_ms;
    const double bridge_provided_default_tf_period = 20.0 * cmn_ms;

    /*! Replaces characters that can't be used in ROS namespace by '_'
      (e.g. space, '.', '-') */
    void clean_namespace(std::string & _ros_namespace);

    /*! Extract the CRTK command from an mts command that might
      include a namespace.  E.g. if the full command is
      "gripper/measured_js", the corresponding crtk command is
      "measures_js" */
    void get_crtk_command(const std::string & _full_command,
                          std::string & _crtk_command);

    /*! Dummy ROS init for bridge components created using dynamic
      creation and no access to argc and argv. */
    ros::NodeHandle * ros_init(const std::string & name);
}

#endif // _cisst_ros_crtk_h
