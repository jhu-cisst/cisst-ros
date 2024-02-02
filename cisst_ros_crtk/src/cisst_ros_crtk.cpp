/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/cisst_ros_crtk.h>

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
