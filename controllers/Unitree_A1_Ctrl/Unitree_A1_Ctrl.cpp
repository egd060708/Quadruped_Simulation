#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include "LegCtrl.h"
#include "RobotParams.h"

using namespace webots;
using namespace Quadruped;
using namespace std;

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // get motor
  Motor *lf_hip = robot->getMotor("FL_hip_joint");
  Motor *lf_thigh = robot->getMotor("FL_thigh_joint");
  Motor *lf_calf = robot->getMotor("FL_calf_joint");
  lf_hip->setPosition(INFINITY);
  lf_thigh->setPosition(INFINITY);
  lf_calf->setPosition(INFINITY);
  lf_hip->setVelocity(0);
  lf_thigh->setVelocity(0);
  lf_calf->setVelocity(0);
  Motor *rf_hip = robot->getMotor("FR_hip_joint");
  Motor *rf_thigh = robot->getMotor("FR_thigh_joint");
  Motor *rf_calf = robot->getMotor("FR_calf_joint");
  // rf_hip->setPosition(INFINITY);
  // rf_thigh->setPosition(INFINITY);
  // rf_calf->setPosition(INFINITY);
  // rf_hip->setVelocity(0);
  // rf_thigh->setVelocity(0);
  // rf_calf->setVelocity(0);
  Motor *lb_hip = robot->getMotor("RL_hip_joint");
  Motor *lb_thigh = robot->getMotor("RL_thigh_joint");
  Motor *lb_calf = robot->getMotor("RL_calf_joint");
  // lb_hip->setPosition(INFINITY);
  // lb_thigh->setPosition(INFINITY);
  // lb_calf->setPosition(INFINITY);
  // lb_hip->setVelocity(0);
  // lb_thigh->setVelocity(0);
  // lb_calf->setVelocity(0);
  Motor *rb_hip = robot->getMotor("RR_hip_joint");
  Motor *rb_thigh = robot->getMotor("RR_thigh_joint");
  Motor *rb_calf = robot->getMotor("RR_calf_joint");
  // rb_hip->setPosition(INFINITY);
  // rb_thigh->setPosition(INFINITY);
  // rb_calf->setPosition(INFINITY);
  // rb_hip->setVelocity(0);
  // rb_thigh->setVelocity(0);
  // rb_calf->setVelocity(0);

  // get position sensor
  PositionSensor *lf_hip_ps = robot->getPositionSensor("FL_hip_joint_sensor");
  PositionSensor *lf_thigh_ps = robot->getPositionSensor("FL_thigh_joint_sensor");
  PositionSensor *lf_calf_ps = robot->getPositionSensor("FL_calf_joint_sensor");
  lf_hip_ps->enable(timeStep);
  lf_thigh_ps->enable(timeStep);
  lf_calf_ps->enable(timeStep);
  PositionSensor *rf_hip_ps = robot->getPositionSensor("FR_hip_joint_sensor");
  PositionSensor *rf_thigh_ps = robot->getPositionSensor("FR_thigh_joint_sensor");
  PositionSensor *rf_calf_ps = robot->getPositionSensor("FR_calf_joint_sensor");
  rf_hip_ps->enable(timeStep);
  rf_thigh_ps->enable(timeStep);
  rf_calf_ps->enable(timeStep);
  PositionSensor *lb_hip_ps = robot->getPositionSensor("RL_hip_joint_sensor");
  PositionSensor *lb_thigh_ps = robot->getPositionSensor("RL_thigh_joint_sensor");
  PositionSensor *lb_calf_ps = robot->getPositionSensor("RL_calf_joint_sensor");
  lb_hip_ps->enable(timeStep);
  lb_thigh_ps->enable(timeStep);
  lb_calf_ps->enable(timeStep);
  PositionSensor *rb_hip_ps = robot->getPositionSensor("RR_hip_joint_sensor");
  PositionSensor *rb_thigh_ps = robot->getPositionSensor("RR_thigh_joint_sensor");
  PositionSensor *rb_calf_ps = robot->getPositionSensor("RR_calf_joint_sensor");
  rb_hip_ps->enable(timeStep);
  rb_thigh_ps->enable(timeStep);
  rb_calf_ps->enable(timeStep);


  // self controll classes
  Leg lf_leg_obj(Quadruped::L1,Quadruped::L2,Quadruped::L3,1);
  LegCtrl lf_leg_ctrl(&lf_leg_obj,timeStep);
  Eigen::Vector3f lastAngle = Eigen::Vector3f::Zero();//零初始化一个上次角度
  Eigen::Vector3f initEndState(0,0.0838,-0.27);
  lf_leg_ctrl.setEndPositionTar(initEndState);

  Leg rf_leg_obj(Quadruped::L1,Quadruped::L2,Quadruped::L3,-1);
  LegCtrl rf_leg_ctrl(&rf_leg_obj,timeStep);
  Eigen::Vector3f lastAngle1 = Eigen::Vector3f::Zero();//零初始化一个上次角度
  Eigen::Vector3f initEndState1(0,-0.0838,-0.27);
  rf_leg_ctrl.setEndPositionTar(initEndState1);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1)
  {
    // lf_hip->setTorque(0);
    // lf_thigh->setTorque(0);
    // lf_calf->setTorque(0);
    // rf_hip->setTorque(0);
    // rf_thigh->setTorque(0);
    // rf_calf->setTorque(0);
    // lb_hip->setTorque(0);
    // lb_thigh->setTorque(0);
    // lb_calf->setTorque(0);
    // rb_hip->setTorque(0);
    // rb_thigh->setTorque(0);
    // rb_calf->setTorque(0);


    Eigen::Vector3f positionSensor;
    positionSensor(0) = lf_hip_ps->getValue();
    positionSensor(1) = lf_thigh_ps->getValue();
    positionSensor(2) = lf_calf_ps->getValue();
    Eigen::Vector3f motorSpeed = (positionSensor - lastAngle) / (0.001f*static_cast<float>(timeStep));
    lastAngle = positionSensor;
    lf_leg_ctrl.updateMotorAng(positionSensor);
    lf_leg_ctrl.updateMotorVel(motorSpeed);
    lf_leg_ctrl.legStateCal();
    lf_leg_ctrl.legCtrlForce();
    lf_hip->setTorque(upper::constrain(lf_leg_obj.targetJoint.Torque(0),43));
    lf_thigh->setTorque(upper::constrain(lf_leg_obj.targetJoint.Torque(1),43));
    lf_calf->setTorque(upper::constrain(lf_leg_obj.targetJoint.Torque(2),43));
    IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    cout<<"current:"<<endl;
    cout << lf_leg_obj.currentLeg.Position.format(CommaInitFmt) << "\n" << endl;
    cout<<"target:"<<endl;
    cout << lf_leg_obj.targetLeg.Position.format(CommaInitFmt) << "\n" << endl;

    Eigen::Vector3f positionSensor1;
    positionSensor1(0) = rf_hip_ps->getValue();
    positionSensor1(1) = rf_thigh_ps->getValue();
    positionSensor1(2) = rf_calf_ps->getValue();
    Eigen::Vector3f motorSpeed1 = (positionSensor1 - lastAngle1) / (0.001f*static_cast<float>(timeStep));
    lastAngle1 = positionSensor1;
    rf_leg_ctrl.updateMotorAng(positionSensor1);
    rf_leg_ctrl.updateMotorVel(motorSpeed1);
    rf_leg_ctrl.legStateCal();
    rf_leg_ctrl.legCtrlForce();
    rf_hip->setTorque(upper::constrain(rf_leg_obj.targetJoint.Torque(0),43));
    rf_thigh->setTorque(upper::constrain(rf_leg_obj.targetJoint.Torque(1),43));
    rf_calf->setTorque(upper::constrain(rf_leg_obj.targetJoint.Torque(2),43));


    // Matrix3f testm;
    // testm << 1,2,3,4,5,6,7,8,9;
    // Vector3f testv(1,2,3);
    // Vector3f testv1 = testm * testv;
    // cout << testv1.format(CommaInitFmt) << "\n" << endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
