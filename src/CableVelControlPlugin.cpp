#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class CableVelControlPlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        physics::LinkPtr link;
        event::ConnectionPtr updateConnection;

        ignition::math::Vector3d targetPosition;
        ignition::math::Vector3d error;
        ignition::math::Vector3d integral;
        ignition::math::Vector3d prevError;
        ignition::math::Vector3d derivative;
        ignition::math::Vector3d controlInput;

        double kP;
        double kI;
        double kD;

        std::shared_ptr<rclcpp::Node> rosNode;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr targetSub;

    public:
        CableVelControlPlugin() : ModelPlugin(), kP(5.0), kI(0.01), kD(0.01) {}

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            this->model = _model;

            if (!rclcpp::ok())
            {
                RCLCPP_FATAL(rclcpp::get_logger("CableVelControlPlugin"), "ROS2 is not initialized. Plugin will not work.");
                return;
            }

            this->link = this->model->GetLink("end_sphere");
            if (!this->link)
            {
                RCLCPP_FATAL(rclcpp::get_logger("CableVelControlPlugin"), "Link end_sphere not found. Plugin will not work.");
                return;
            }

            this->targetPosition = this->link->WorldPose().Pos();
            this->error = this->targetPosition - this->link->WorldPose().Pos();
            this->integral = ignition::math::Vector3d(0, 0, 0);
            this->prevError = ignition::math::Vector3d(0, 0, 0);

            this->rosNode = gazebo_ros::Node::Get(_sdf);

            this->targetSub = this->rosNode->create_subscription<geometry_msgs::msg::Point>(
                "/cable_target_position", 10,
                [this](geometry_msgs::msg::Point::SharedPtr msg) {
                    this->targetPosition.Set(msg->x, msg->y, msg->z);
                });

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&CableVelControlPlugin::OnUpdate, this));
        }

        void OnUpdate()
        {
            this->error = this->targetPosition - this->link->WorldPose().Pos();
            this->integral += this->error;
            this->derivative = this->error - this->prevError;
            this->controlInput = this->kP * this->error + this->kI * this->integral + this->kD * this->derivative;

            this->link->SetLinearVel(this->controlInput);

            this->prevError = this->error;

            RCLCPP_INFO(this->rosNode->get_logger(), "Current position: [%.2f, %.2f, %.2f]", 
                        this->link->WorldPose().Pos().X(),
                        this->link->WorldPose().Pos().Y(),
                        this->link->WorldPose().Pos().Z());
            RCLCPP_INFO(this->rosNode->get_logger(), "Control input: [%.2f, %.2f, %.2f]", 
                        this->controlInput.X(),
                        this->controlInput.Y(),
                        this->controlInput.Z());
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(CableVelControlPlugin)
}
