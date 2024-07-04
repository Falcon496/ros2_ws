#include "simple_steer.hpp"
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/msgs/twist.pb.h>

namespace iginition_plugin_steer
{

SteerControler::SteerControler()
{
  CreateIgnitionIf();
}

SteerControler::~SteerControler()
{
}

// シミュレーション起動時
void SteerControler::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  (void)_ecm;
  (void)_eventMgr;
  model_ = ignition::gazebo::Model(_entity);

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  sdf::ElementPtr steer1_joint_elem = ptr->GetElement("steer1_joint");
  sdf::ElementPtr steer2_joint_elem = ptr->GetElement("steer2_joint");
  sdf::ElementPtr steer3_joint_elem = ptr->GetElement("steer3_joint");
  sdf::ElementPtr steer4_joint_elem = ptr->GetElement("steer4_joint");
  sdf::ElementPtr wheel1_joint_elem = ptr->GetElement("wheel1_joint");
  sdf::ElementPtr wheel2_joint_elem = ptr->GetElement("wheel2_joint");
  sdf::ElementPtr wheel3_joint_elem = ptr->GetElement("wheel3_joint");
  sdf::ElementPtr wheel4_joint_elem = ptr->GetElement("wheel4_joint");
  if (steer1_joint_elem && steer2_joint_elem && steer3_joint_elem && steer4_joint_elem) {
    steer1_joint_name_ = steer1_joint_elem->Get<std::string>();
    steer2_joint_name_ = steer2_joint_elem->Get<std::string>();
    steer3_joint_name_ = steer3_joint_elem->Get<std::string>();
    steer4_joint_name_ = steer4_joint_elem->Get<std::string>();
  } else {
    ignerr << "sdf steer_joint not found" << std::endl;
  }
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
void SteerControler::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    (void)_info;
    (void)_ecm;
    // model_.JointByName(_ecm, "joint_name");で、過去に設定したジョイントの設定が取れます。
    ignition::gazebo::Entity steer1_joint = model_.JointByName(_ecm, steer1_joint_name_);
    ignition::gazebo::Entity steer2_joint = model_.JointByName(_ecm, steer2_joint_name_);
    ignition::gazebo::Entity steer3_joint = model_.JointByName(_ecm, steer3_joint_name_);
    ignition::gazebo::Entity steer4_joint = model_.JointByName(_ecm, steer4_joint_name_);
    ignition::gazebo::Entity weel1_joint = model_.JointByName(_ecm, wheel1_joint_name_);
    ignition::gazebo::Entity weel2_joint = model_.JointByName(_ecm, wheel2_joint_name_);
    ignition::gazebo::Entity weel3_joint = model_.JointByName(_ecm, wheel3_joint_name_);
    ignition::gazebo::Entity weel4_joint = model_.JointByName(_ecm, wheel4_joint_name_);
    if (steer1_joint == ignition::gazebo::kNullEntity){
      ignerr << steer1_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (steer2_joint == ignition::gazebo::kNullEntity){
      ignerr << steer2_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (steer3_joint == ignition::gazebo::kNullEntity){
      ignerr << steer3_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (steer4_joint == ignition::gazebo::kNullEntity){
      ignerr << steer4_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (weel1_joint == ignition::gazebo::kNullEntity){
      ignerr << wheel1_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (weel2_joint == ignition::gazebo::kNullEntity){
      ignerr << wheel2_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (weel3_joint == ignition::gazebo::kNullEntity){
      ignerr << wheel3_joint_name_ <<" not found" << std::endl;
      return;
    }
    if (weel4_joint == ignition::gazebo::kNullEntity){
      ignerr << wheel4_joint_name_ <<" not found" << std::endl;
      return;
    }

    // 空でないときはJointVelocityCmd()で上書きをします。そうでないときはCreateComponent()でリストに追加
    auto ang1 = _ecm.Component<ignition::gazebo::components::JointPosition>(steer1_joint);
    auto ang2 = _ecm.Component<ignition::gazebo::components::JointPosition>(steer2_joint);
    auto ang3 = _ecm.Component<ignition::gazebo::components::JointPosition>(steer3_joint);
    auto ang4 = _ecm.Component<ignition::gazebo::components::JointPosition>(steer4_joint);
    auto vel1 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(weel1_joint);
    auto vel2 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(weel2_joint);
    auto vel3 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(weel3_joint);
    auto vel4 = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(weel4_joint);
    if (ang1 != nullptr) {
      *ang1 = ignition::gazebo::components::JointPosition({steer_ang_});
    }
    else {
      _ecm.CreateComponent(steer1_joint, ignition::gazebo::components::JointPosition({steer_ang_}));
    }
    if (ang2 != nullptr) {
      *ang2 = ignition::gazebo::components::JointPosition({steer_ang_});
    }
    else {
      _ecm.CreateComponent(steer2_joint, ignition::gazebo::components::JointPosition({steer_ang_}));
    }
    if (ang3 != nullptr) {
      *ang3 = ignition::gazebo::components::JointPosition({steer_ang_});
    }
    else {
      _ecm.CreateComponent(steer3_joint, ignition::gazebo::components::JointPosition({steer_ang_}));
    }
    if (ang4 != nullptr) {
      *ang4 = ignition::gazebo::components::JointPosition({steer_ang_});
    }
    else {
      _ecm.CreateComponent(steer4_joint, ignition::gazebo::components::JointPosition({wheel_speed_}));
    }
    if (vel1 != nullptr) {
      *vel1 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(weel1_joint, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
    if (vel2 != nullptr) {
      *vel2 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(weel2_joint, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
    if (vel3 != nullptr) {
      *vel3 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(weel3_joint, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
    if (vel4 != nullptr) {
      *vel4 = ignition::gazebo::components::JointVelocityCmd({wheel_speed_});
    }
    else {
      _ecm.CreateComponent(weel4_joint, ignition::gazebo::components::JointVelocityCmd({wheel_speed_}));
    }
 }


// ワールド更新中
void SteerControler::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}

// ワールド更新後(PostUpdate())、テレメトリーの出力等を行います。
void SteerControler::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  (void)_info;
  (void)_ecm;
}


void SteerControler::CreateIgnitionIf(void){
  this->node_.Subscribe("steer_speed", &SteerControler::OnSpeedMessage, this);
}

// 受信コールバックに登録したOnSpeedMessage()で角速度の指令値を受け取ります。
void SteerControler::OnSpeedMessage(const ignition::msgs::Twist &msg)
{
  wheel_speed_ = std::sqrt(msg.linear().x()*msg.linear().x() + msg.linear().y() * msg.linear().y());
  steer_ang_ = std::atan2(msg.linear().y(), msg.linear().x());
}

}

#include <ignition/plugin/Register.hh>
IGNITION_ADD_PLUGIN(
    iginition_plugin_steer::SteerControler,
    ignition::gazebo::System,
    iginition_plugin_steer::SteerControler::ISystemConfigure,
    iginition_plugin_steer::SteerControler::ISystemPreUpdate,
    iginition_plugin_steer::SteerControler::ISystemUpdate,
    iginition_plugin_steer::SteerControler::ISystemPostUpdate)
