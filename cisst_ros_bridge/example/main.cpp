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


/* usage:
  rosrun cisst_ros_bridge example_bridge

  Then in different terminals:

  [1] rostopic echo /sawROSExample/sum_of_elements_1

  [2] rostopic pub /sawROSExample/set_value_1 cisst_msgs/DoubleVec "data: [0, 1, 2, 4]" -1
  >> result in sum_of_elements_1 should be 7


*/

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include "cisst_ros_bridge/mtsROSBridge.h"

class TestComponent: public mtsTaskPeriodic
{
public:
    TestComponent(void):
        mtsTaskPeriodic("testComponent", 5.0 * cmn_ms, 256),
        Value1(4),
        Value2(4),
        Value3(4),
        OldValue3(4)
    {
        Value1.fill(0.0);
        Value2.fill(0.0);
        Value3.fill(0.0);
        OldValue3.fill(0.0);

        StateTable.AddData(Value1, "value1");
        StateTable.AddData(Value2, "value2");
        StateTable.AddData(Value3, "value3");
        mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("provided");
        interfaceProvided->AddCommandReadState(StateTable, Value1, "GetValue1");
        interfaceProvided->AddCommandReadState(StateTable, Value2, "GetValue2");
        interfaceProvided->AddCommandReadState(StateTable, Value3, "GetValue3");
        interfaceProvided->AddCommandWrite(&TestComponent::SetValue1, this,
                                           "SetValue1", Value1);
        interfaceProvided->AddCommandWrite(&TestComponent::SetValue2, this,
                                           "SetValue2", Value2);

        mtsInterfaceRequired * InterfacesRequired = AddInterfaceRequired("required");
        InterfacesRequired->AddFunction("SumOfElements1", SumOfElements1);
        InterfacesRequired->AddFunction("ValueChanged2", ValueChanged2);
        InterfacesRequired->AddEventHandlerWrite(&TestComponent::SetValue1, this, "EventValue1");
        InterfacesRequired->AddEventHandlerVoid(&TestComponent::Reset, this, "EventReset");
        InterfacesRequired->AddFunction("GetValue3", GetValue3);
    }

    void Configure(const std::string &) {
    }

    void Startup(void) {
    }

    void Run(void) {
        ProcessQueuedCommands();
        ProcessQueuedEvents();

        GetValue3(Value3);
        Value3.resize(4);
        if (Value3 != OldValue3) {
            std::cout << "Value3 " << Value3 << std::endl;
            OldValue3 = Value3;
        }
    }

    void Cleanup(void) {
    }

protected:
    void SetValue1(const Eigen::VectorXd& newValue) {
        Value1 = newValue;
        SumOfElements1(Value1.sum());
    }

    void SetValue2(const Eigen::VectorXd& newValue) {
        Value2 = newValue;
        ValueChanged2();
    }

    void Reset(void) {
        Value1.fill(0.0);
        Value2.fill(0.0);
        ValueChanged2();
    }

    Eigen::VectorXd Value1;
    Eigen::VectorXd Value2;
    Eigen::VectorXd Value3;
    Eigen::VectorXd OldValue3;

    mtsFunctionRead GetValue3;
    mtsFunctionWrite SumOfElements1;
    mtsFunctionVoid ValueChanged2;
};

int main(int argc, char **argv)
{
    cisst_ral::ral ral(argc, argv, "cisst_ros_bridge_example");
    cisst_ral::node_ptr_t node = ral.node();

    mtsComponentManager * manager = mtsManagerLocal::GetInstance();

    TestComponent testComponent;
    manager->AddComponent(&testComponent);

    mtsROSBridge bridge("publisher", 5.0 * cmn_ms, node);
    bridge.PerformsSpin(true);

    bridge.AddPublisherFromCommandRead<Eigen::VectorXd, CISST_RAL_MSG(cisst_msgs, DoubleVec)>
        ("required", "GetValue1", "sawROSExample/get_value_1");

    bridge.AddPublisherFromCommandRead<Eigen::VectorXd, CISST_RAL_MSG(cisst_msgs, DoubleVec)>
        ("required", "GetValue2", "sawROSExample/get_value_2");

    bridge.AddSubscriberToCommandWrite<Eigen::VectorXd, CISST_RAL_MSG(cisst_msgs, DoubleVec)>
        ("required", "SetValue1", "sawROSExample/set_value_1");

    bridge.AddSubscriberToCommandWrite<Eigen::VectorXd, CISST_RAL_MSG(cisst_msgs, DoubleVec)>
        ("required", "SetValue2", "sawROSExample/set_value_2");

    bridge.AddPublisherFromCommandWrite<double, CISST_RAL_MSG(std_msgs, Float32)>
        ("provided", "SumOfElements1", "sawROSExample/sum_of_elements_1");

    bridge.AddPublisherFromCommandVoid
        ("provided", "ValueChanged2", "sawROSExample/value_changed_2");

    bridge.AddSubscriberToEventWrite<Eigen::VectorXd, CISST_RAL_MSG(cisst_msgs, DoubleVec)>
        ("provided", "EventValue1", "sawROSExample/set_value_1_event");

    bridge.AddSubscriberToEventVoid
        ("provided", "EventReset", "sawROSExample/reset_event");

    bridge.AddSubscriberToCommandRead<Eigen::VectorXd, CISST_RAL_MSG(cisst_msgs, DoubleVec)>
        ("provided", "GetValue3", "sawROSExample/set_value_3");

    manager->AddComponent(&bridge);

    manager->Connect(bridge.GetName(), "required",
                     testComponent.GetName(), "provided");

    manager->Connect(testComponent.GetName(), "required",
                     bridge.GetName(), "provided");

    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    std::cout << "Hit Ctrl-c to quit" << std::endl;
    cisst_ral::spin(node);

    manager->KillAllAndWait(2.0 * cmn_s);
    manager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
