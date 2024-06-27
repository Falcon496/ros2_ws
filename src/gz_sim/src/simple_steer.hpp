#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>

namespace iginition_plugin_lecture
{
  class RotateAxis:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: 
        RotateAxis();
        ~RotateAxis() override;
        void Configure(const ignition::gazebo::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            ignition::gazebo::EntityComponentManager &_ecm,
            ignition::gazebo::EventManager &_eventMgr) override;
        void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
        void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
        void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
    private:
      void CreateIgnitionIf(void);
      void OnSpeedMessage(const ignition::msgs::Twist &msg);

      ignition::gazebo::Model model_;
      ignition::transport::Node node_;
      std::string wheel1_joint_name_{""};
      std::string wheel2_joint_name_{""};
      std::string wheel3_joint_name_{""};
      std::string wheel4_joint_name_{""};
      float wheel_speed_{0.0f};
      float wheel_rad_{0.0f};
  };
}
