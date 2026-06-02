#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo
{

class GroundTruthOdomPlugin : public ModelPlugin
{
public:

    GroundTruthOdomPlugin() : ModelPlugin()
    {
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      model_ = model;

      link_ = model_->GetLink("base_link");

      if (!link_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[GroundTruthOdomPlugin] base_link não encontrado!");
        return;
      }

      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      node_ = std::make_shared<rclcpp::Node>("ground_truth_odom");

      pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
          "/odom_gt", 10);

      update_connection_ =
        event::Events::ConnectWorldUpdateBegin(
            std::bind(&GroundTruthOdomPlugin::OnUpdate, this));

      last_time_ = model_->GetWorld()->SimTime();

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[GroundTruthOdomPlugin] Inicializado.");
    }

private:

    void OnUpdate()
    {
        rclcpp::spin_some(node_);

        auto pose = link_->WorldPose();
        auto lin_vel = link_->WorldLinearVel();
        auto ang_vel = link_->WorldAngularVel();

        nav_msgs::msg::Odometry msg;

        msg.header.stamp = node_->now();
        msg.header.frame_id = "world";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = pose.Pos().X();
        msg.pose.pose.position.y = pose.Pos().Y();
        msg.pose.pose.position.z = pose.Pos().Z();

        msg.pose.pose.orientation.x = pose.Rot().X();
        msg.pose.pose.orientation.y = pose.Rot().Y();
        msg.pose.pose.orientation.z = pose.Rot().Z();
        msg.pose.pose.orientation.w = pose.Rot().W();

        msg.twist.twist.linear.x = lin_vel.X();
        msg.twist.twist.linear.y = lin_vel.Y();
        msg.twist.twist.linear.z = lin_vel.Z();

        msg.twist.twist.angular.x = ang_vel.X();
        msg.twist.twist.angular.y = ang_vel.Y();
        msg.twist.twist.angular.z = ang_vel.Z();

        pub_->publish(msg);
    }

private:

    physics::ModelPtr model_;
    physics::LinkPtr link_;

    event::ConnectionPtr update_connection_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

    gazebo::common::Time last_time_;
};

GZ_REGISTER_MODEL_PLUGIN(GroundTruthOdomPlugin)

}
