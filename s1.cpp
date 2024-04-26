#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <random>
#include <cmath>
#include <iomanip>
#include <iostream>
class BrownianMotionUpdater {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_p1_; // Subscriber to topic_p1
    ros::Subscriber sub_p2_; // Subscriber to topic_p2
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_int_distribution<> distrib_;

public:
    BrownianMotionUpdater()
        : nh_(), gen_(rd_()), distrib_(1, 2) {
        sub_p1_ = nh_.subscribe("topic_p1", 1, &BrownianMotionUpdater::callback, this);
        sub_p2_ = nh_.subscribe("topic_p2", 1, &BrownianMotionUpdater::callback, this);
    }
    void callback(const std_msgs::Float64& message_holder) {
        int chosen = distrib_(gen_);
        visualizeTrajectory(chosen, message_holder.data);
    }

    void visualizeTrajectory(int chosen, double current_value) {
        char chosen_char = (chosen == 1 ? '1' : '2');
        // Center the trajectory around 0 and scale by 2 for visibility
        int center_position = 40; // Center position on your console
        int num_spaces = center_position + static_cast<int>(current_value * 2); // Scale factor of 2 for better visibility
        // Clamp the number of spaces to avoid overflow on console
        num_spaces = std::max(0, num_spaces);
        num_spaces = std::min(80, num_spaces); // Assuming console width of 80 characters
        std::string spaces(num_spaces, ' ');
        std::cout << spaces << chosen_char << std::endl;
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "s1_ros");
    BrownianMotionUpdater updater;
    ros::spin();
    return 0;
}
