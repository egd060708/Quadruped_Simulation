#pragma once

#include "Leg.h"
#include "../../libraries/myLibs/PIDmethod.h"

namespace Quadruped
{

    class LegCtrl
    {
    private:
        Leg *legObject;
        PIDmethod jPid[3]; // 用于关节位控的控制器
        PIDmethod lPid[3]; // 用于末端力控的控制器
    public:
        LegCtrl(Leg *_l, uint32_t timeStep) : legObject(_l)
        {
            for (auto p : jPid)
            {
                p.PID_Init(Common, static_cast<double>(0.001 * timeStep));
            }
            for (auto p : lPid)
            {
                p.PID_Init(Common, static_cast<double>(0.001 * timeStep));
            }
            lPid[0].Params_Config(150., 0, -2, 0, 100, -100);
            lPid[1].Params_Config(250., 0, -5, 0, 100, -100);
            lPid[2].Params_Config(80., 0, -1, 0, 100, -100);
        }
        // 更新电机观测值
        void updateMotorAng(Vector3f _a);
        void updateMotorVel(Vector3f _w);
        void updateMotorTau(Vector3f _t);
        // 更新末端目标值
        void setEndPositionTar(Vector3f _p);
        void setEndVelocityTar(Vector3f _v);
        void setEndForceTar(Vector3f _f);
        // 对腿部整体进行状态计算
        void legStateCal();
        // 腿部控制执行
        void legCtrlPosition(); // 直接对腿部电机进行位控
        void legCtrlForce();    // 对腿部末端位置进行力控
    };

    void LegCtrl::updateMotorAng(Vector3f _a)
    {
        legObject->updateJointAng(_a);
    }

    void LegCtrl::updateMotorVel(Vector3f _w)
    {
        legObject->updateJointVel(_w);
    }

    void LegCtrl::updateMotorTau(Vector3f _t)
    {
        legObject->updateJointTau(_t);
    }

    void LegCtrl::setEndPositionTar(Vector3f _p)
    {
        legObject->setTargetLegPositon(_p);
    }

    void LegCtrl::setEndVelocityTar(Vector3f _v)
    {
        legObject->setTargetLegVelocity(_v);
    }

    void LegCtrl::setEndForceTar(Vector3f _f)
    {
        legObject->setTargetLegForce(_f);
    }

    void LegCtrl::legStateCal()
    {
        legObject->legJacobi_Cal();
        legObject->legFK_Cal();
        legObject->legIK_Cal();
    }

    void LegCtrl::legCtrlPosition()
    {
        for (int i = 0; i < 3; i++)
        {
            jPid[i].target = legObject->targetJoint.Angle(i);
            jPid[i].current = legObject->currentJoint.Angle(i);
            jPid[i].Adjust(0, legObject->currentJoint.Velocity(i));
            legObject->targetJoint.Torque(i) = jPid[i].out;
        }
    }

    void LegCtrl::legCtrlForce()
    {
        Vector3f tmp;
        for (int i = 0; i < 3; i++)
        {
            lPid[i].target = legObject->targetLeg.Position(i);
            lPid[i].current = legObject->currentLeg.Position(i);
            lPid[i].Adjust(0, legObject->currentLeg.Velocity(i));
            tmp(i) = lPid[i].out;
        }
        legObject->setTargetLegForce(tmp);
    }
}