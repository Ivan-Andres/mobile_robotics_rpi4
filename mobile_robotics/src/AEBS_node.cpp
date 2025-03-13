#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <limits>

class AEBSNode : public rclcpp::Node {
public:
    AEBSNode() : Node("AEBS_node"), prev_distance(std::numeric_limits<float>::infinity()) {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AEBSNode::scan_callback, this, std::placeholders::_1));
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_break", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo AEBS inicializado.");
    }

private:
    double get_ttc_threshold(double velocity) {
        // Interpolación lineal entre (0.02 m/s, 8s) y (0.2 m/s, 2s)
        if (velocity <= 0.02) return 8.0;
        if (velocity >= 0.2) return 2.0;
        return 8.0 - ((velocity - 0.02) * (8.0 - 2.0) / (0.2 - 0.02));
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_distance = std::numeric_limits<float>::infinity();
        int start_index = (3 * msg->ranges.size() / 4) - 10;
        int end_index = (3 * msg->ranges.size() / 4) + 10;

        for (int i = start_index; i <= end_index; ++i) {
            float distance = msg->ranges[i];

            if (distance > 0.0 && distance < 10.0) { // Filtrar datos inválidos
                if (distance < min_distance) {
                    min_distance = distance - 0.0;

                    if (min_distance < 0.0) {
                        min_distance = 0.0;
                    }
                }
            }
        }

        double current_time = this->now().seconds();
        if (!std::isinf(prev_distance)) {
            dt = current_time - prev_time;
            ds = prev_distance - min_distance;
            if (dt > 0) {
                v_lidar = ds/ dt; // Estimación de velocidad
            }
        }

        RCLCPP_INFO(this->get_logger(), "Delta t: %.6f s, Distancia mínima: %.6f m, delta distancia: %.6f m, velocidad: %.6f m/s", dt, min_distance,ds, v_lidar);

        prev_distance = min_distance;
        prev_time = current_time;

        geometry_msgs::msg::Twist cmd_vel_msg;

        double ttc_threshold = get_ttc_threshold(v_lidar);

        RCLCPP_WARN(this->get_logger(), "TTC = %.2f s, threshold = %.2f s", ttc, ttc_threshold );
        ttc = min_distance / 0.8;
        if (ttc < 2.0 && ttc >= 0) {
            RCLCPP_WARN(this->get_logger(), "ttc menor a 2s. Frenando de emergencia.");
            cmd_vel_msg.linear.x = -0.4;
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel_msg);
            return;
        }
        // } else if (min_distance <= 0.2) {
        //     RCLCPP_WARN(this->get_logger(), "Distancia menor a 0.2m. Frenando de emergencia.");
        //     cmd_vel_msg.linear.x = -0.4;
        //     cmd_vel_msg.angular.z = 0.0;
        //     cmd_vel_pub_->publish(cmd_vel_msg);
        //     return;
        // }

        RCLCPP_INFO(this->get_logger(), "Velocidad segura. No se requiere frenado.");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    double ttc_threshold;
    double v_lidar;
    double prev_distance;
    double prev_time;
    double dt;
    double ds;
    double ttc;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AEBSNode>());
    rclcpp::shutdown();
    return 0;
}
