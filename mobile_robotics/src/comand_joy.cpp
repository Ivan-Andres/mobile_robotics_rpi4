#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyTwistNode : public rclcpp::Node {
public:
    JoyTwistNode() : Node("joy_twist") {
        // Suscripción al topic /joy
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyTwistNode::joy_callback, this, std::placeholders::_1));

        // Publicación en /joy_twist
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_joy", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo comand_joy iniciado.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        int boton_hombre_muerto = msg->buttons[5]; // Botón 5
        double eje_velocidad = msg->axes[3];       // Eje 4
        double eje_direccion = msg->axes[2];       // Eje 4

        if (boton_hombre_muerto) {

            // Crear mensaje de tipo Twist
            auto twist_msg = geometry_msgs::msg::Twist();

            twist_msg.linear.x = eje_velocidad;
            twist_msg.angular.z = eje_direccion;

            publisher_->publish(twist_msg);

        }
        
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTwistNode>());
    rclcpp::shutdown();
    return 0;
}
