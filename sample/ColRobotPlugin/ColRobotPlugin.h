/**
   Sample Robot motion controller for the JVRC robot model.
   This program was ported from the "RobotTorqueController.h" sample of Choreonoid.
*/

#ifndef RobotTorqueControllerRTC_H
#define RobotTorqueControllerRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/MultiValueSeq>
#include <vector>

class ColRobotPlugin : public RTC::DataFlowComponentBase
{
public:
    ColRobotPlugin(RTC::Manager* manager);
    ~ColRobotPlugin();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_angle;
    RTC::InPort<RTC::TimedDoubleSeq> m_angleIn;

    // DataOutPort declaration
    RTC::TimedDoubleSeq m_torque;
    RTC::OutPort<RTC::TimedDoubleSeq> m_torqueOut;

private:
    cnoid::MultiValueSeqPtr qseq;
    std::vector<double> q0;
    cnoid::MultiValueSeq::Frame oldFrame;
    int currentFrame;
    double timeStep_;
};

extern "C"
{
    DLL_EXPORT void RobotTorqueControllerRTCInit(RTC::Manager* manager);
};

#endif
