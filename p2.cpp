#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <cstdlib>
// Class to sample values from a normal distribution and publish them to topic_p2
class NormalDistributionSampler {
public:
    // Constructor: initializes ROS communication and sets the publication rate
    NormalDistributionSampler(ros::NodeHandle& nati) : nh_(nati), naptime(0.1) {
        // Advertise the ROS topic "topic_p2" with a message queue size of 1
        my_publisher_object = nh_.advertise<std_msgs::Float64>("topic_p2", 1);
    }

    // Main loop: generates and publishes samples as long as ROS is running
    void run() {
        while (ros::ok()) {
            double sample = gen_norm_sample();  // Generate a normally distributed sample
            publish(sample);                   // Publish the sample
            ROS_INFO("Published to topic_p2: %f", sample); // Log the value
            naptime.sleep();                   // Wait to maintain the 10Hz rate
        }
    }

private:
    ros::NodeHandle nh_;                 // Node handle for managing ROS system resources
    ros::Publisher my_publisher_object;  // ROS publisher
    ros::Rate naptime;                   // Rate object to manage publish rate

    // Generates a normally distributed sample using the Box-Muller transform
    double gen_norm_sample() {
        double u = static_cast<double>(rand()) / RAND_MAX; // Uniform random value between 0 and 1
        double v = static_cast<double>(rand()) / RAND_MAX; // Another uniform random value
        double sample = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v) * (10.0 / 3.0);
        if (sample < -10.0) sample = -10.0; // Clamp the sample to minimum -10.0
        else if (sample > 10.0) sample = 10.0; // Clamp the sample to maximum 10.0
        return sample;
    }

    // Publishes the generated sample to the topic
    void publish(double value) {
        std_msgs::Float64 msg;              // Create a Float64 message
        msg.data = value;                   // Set the message data to the generated value
        my_publisher_object.publish(msg);   // Publish the message
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "p2_ros");      // Initialize the ROS node named "p2_ros"
    ros::NodeHandle nati;                 // Create a node handle
    NormalDistributionSampler sampler(nati); // Create the sampler object
    sampler.run();                        // Run the sampler
    return 0;
}
