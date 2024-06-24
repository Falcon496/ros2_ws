#include "simple_steer.hpp"
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/twist.pb.h>

namespace iginition_plugin_lecture
{

RotateAxis::RotateAxis()
{
  CreateIgnitionIf();
}

RotateAxis::~RotateAxis()
{
}

void RotateAxis::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  (void)_ecm;
  (void)_eventMgr;
  model_ = ignition::gazebo::Model(_entity);

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr target1_joint_elem = ptr->GetElement("target1_joint");
  sdf::ElementPtr target2_joint_elem = ptr->GetElement("target2_joint");
  sdf::ElementPtr target3_joint_elem = ptr->GetElement("target3_joint");
  sdf::ElementPtr target4_joint_elem = ptr->GetElement("target4_joint");
  if (target1_joint_elem && target2_joint_elem && target3_joint_elem && target4_joint_elem) {
    target1_joint_name_ = target1_joint_elem->Get<std::string>();
    target2_joint_name_ = target2_joint_elem->Get<std::string>();
    target3_joint_name_ = target3_joint_elem->Get<std::string>();
    target4_joint_name_ = target4_joint_elem->Get<std::string>();
  } else {
    ignerr << "sdf target_joint not found" << std::endl;
  }
}

void RotateAxis::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    (void)_info;
    (void)_ecm;
    ignition::gazebo::Entity joint1 = model_.JointByName(_ecm, target1_joint_name_);
    ignition::gazebo::Entity joint2 = model_.JointByName(_ecm, target2_joint_name_);
    ignition::gazebo::Entity joint3 = model_.JointByName(_ecm, target3_joint_name_);
    ignition::gazebo::Entity joint4 = model_.JointByName(_ecm, target4_joint_name_);
    if (joint1 == ignition::gazebo::kNullEntity){
      ignerr << target1_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (joint2 == ignition::gazebo::kNullEntity){
      ignerr << target2_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (joint3 == ignition::gazebo::kNullEntity){
      ignerr << target3_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (joint4 == ignition::gazebo::kNullEntity){
      ignerr << target4_joint_name_ <<" not found" << std::endl;
      return;
    }

    auto vel1 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint1);
    auto vel2 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint2);
    auto vel3 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint3);
    auto vel4 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint4);
    if (vel1 != nullptr) {
      *vel1 = ignition::gazebo::components::JointVelocityCmd({target_speed_});
    }
    else {
      _ecm.CreateComponent(joint1, ignition::gazebo::components::JointVelocityCmd({target_speed_}));
    }
    if (vel2 != nullptr) {
      *vel2 = ignition::gazebo::components::JointVelocityCmd({target_speed_});
    }
    else {
      _ecm.CreateComponent(joint2, ignition::gazebo::components::JointVelocityCmd({target_speed_}));
    }
    if (vel3 != nullptr) {
      *vel3 = ignition::gazebo::components::JointVelocityCmd({target_speed_});
    }
    else {
      _ecm.CreateComponent(joint3, ignition::gazebo::components::JointVelocityCmd({target_speed_}));
    }
    if (vel4 != nullptr) {
      *vel4 = ignition::gazebo::components::JointVelocityCmd({target_speed_});
    }
    else {
      _ecm.CreateComponent(joint4, ignition::gazebo::components::JointVelocityCmd({target_speed_}));
    }
 }

void RotateAxis::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

void RotateAxis::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

void RotateAxis::CreateIgnitionIf(void){
  this->node_.Subscribe("target_speed", &RotateAxis::OnSpeedMessage, this);
}

void RotateAxis::OnSpeedMessage(const ignition::msgs::Twist &msg)
{
  target_speeds_ = msg;
  target_speed_ = msg.linear().x();
}

}

#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(
    iginition_plugin_lecture::RotateAxis,
    ignition::gazebo::System,
    iginition_plugin_lecture::RotateAxis::ISystemConfigure,
    iginition_plugin_lecture::RotateAxis::ISystemPreUpdate,
    iginition_plugin_lecture::RotateAxis::ISystemUpdate,
    iginition_plugin_lecture::RotateAxis::ISystemPostUpdate)
