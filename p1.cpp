#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <cstdlib>
// Define a class to sample and publish normally distributed values.
class NormalDistributionSampler {
public:
    // Constructor: Initializes the ROS node and sets up the publisher.
    NormalDistributionSampler(ros::NodeHandle& nati) : nh_(nati), naptime(0.1) {
        my_publisher_object = nh_.advertise<std_msgs::Float64>("topic_p1", 1);
    }

    // Continuously generate and publish samples as long as ROS is running.
    void run() {
        while (ros::ok()) {
            double sample = gen_norm_sample(); // Generate a normally distributed sample.
            publish(sample); // Publish the generated sample.
            ROS_INFO("Published to topic_p1: %f", sample); // Log information about the sample.
            naptime.sleep(); // Sleep to maintain the publishing rate.
        }
    }

private:
    ros::NodeHandle nh_; // ROS NodeHandle to manage communication with ROS system.
    ros::Publisher my_publisher_object; // ROS Publisher to publish messages.
    ros::Rate naptime; // Rate at which messages are published.

    // Generate a normally distributed sample using the Box-Muller transformation.
    double gen_norm_sample() {
        double u = static_cast<double>(rand()) / RAND_MAX;
        double v = static_cast<double>(rand()) / RAND_MAX;
        double sample = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v) * (10.0 / 3.0);
        return (sample < -10.0) ? -10.0 : (sample > 10.0) ? 10.0 : sample;
    }

    // Publish the generated value.
    void publish(double value) {
        std_msgs::Float64 msg;
        msg.data = value;
        my_publisher_object.publish(msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "p1_ros"); // Initialize ROS.
    ros::NodeHandle nati; // Create a node handle.
    NormalDistributionSampler sampler(nati); // Create the sampler object.
    sampler.run(); // Start generating and publishing samples.
    return 0;
}
