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

// シミュレーション起動時
void RotateAxis::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  (void)_ecm;
  (void)_eventMgr;
  model_ = ignition::gazebo::Model(_entity);

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr wheel1_joint_elem = ptr->GetElement("wheel1_joint");
  sdf::ElementPtr wheel2_joint_elem = ptr->GetElement("wheel2_joint");
  sdf::ElementPtr wheel3_joint_elem = ptr->GetElement("wheel3_joint");
  sdf::ElementPtr wheel4_joint_elem = ptr->GetElement("wheel4_joint");
  if (wheel1_joint_elem && wheel2_joint_elem && wheel3_joint_elem && wheel4_joint_elem) {
    wheel1_joint_name_ = wheel1_joint_elem->Get<std::string>();
    wheel2_joint_name_ = wheel2_joint_elem->Get<std::string>();
    wheel3_joint_name_ = wheel3_joint_elem->Get<std::string>();
    wheel4_joint_name_ = wheel4_joint_elem->Get<std::string>();
  } else {
    ignerr << "sdf wheel_joint not found" << std::endl;
  }
}

// ワールド更新前、軸の速度の設定などを行う
void RotateAxis::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    (void)_info;
    (void)_ecm;
    // model_.JointByName(_ecm, "joint_name");で、過去に設定したジョイントの設定が取れます。
    ignition::gazebo::Entity joint1 = model_.JointByName(_ecm, wheel1_joint_name_);
    ignition::gazebo::Entity joint2 = model_.JointByName(_ecm, wheel2_joint_name_);
    ignition::gazebo::Entity joint3 = model_.JointByName(_ecm, wheel3_joint_name_);
    ignition::gazebo::Entity joint4 = model_.JointByName(_ecm, wheel4_joint_name_);
    if (joint1 == ignition::gazebo::kNullEntity){
      ignerr << wheel1_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (joint2 == ignition::gazebo::kNullEntity){
      ignerr << wheel2_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (joint3 == ignition::gazebo::kNullEntity){
      ignerr << wheel3_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (joint4 == ignition::gazebo::kNullEntity){
      ignerr << wheel4_joint_name_ <<" not found" << std::endl;
      return;
    }

    // 空でないときはJointVelocityCmd()で上書きをします。そうでないときはCreateComponent()でリストに追加
    auto vel1 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint1);
    auto vel2 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint2);
    auto vel3 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint3);
    auto vel4 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint4);
    if (vel1 != nullptr) {
      *vel1 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(joint1, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
    if (vel2 != nullptr) {
      *vel2 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(joint2, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
    if (vel3 != nullptr) {
      *vel3 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(joint3, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
    if (vel4 != nullptr) {
      *vel4 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(joint4, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
 }


// ワールド更新中
void RotateAxis::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

// ワールド更新後(PostUpdate())、テレメトリーの出力等を行います。
void RotateAxis::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}


void RotateAxis::CreateIgnitionIf(void){
  this->node_.Subscribe("wheel_speed", &RotateAxis::OnSpeedMessage, this);
}

// 受信コールバックに登録したOnSpeedMessage()で角速度の指令値を受け取ります。
void RotateAxis::OnSpeedMessage(const ignition::msgs::Twist &msg)
{
  wheel_speed_ = msg.linear().x();
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
