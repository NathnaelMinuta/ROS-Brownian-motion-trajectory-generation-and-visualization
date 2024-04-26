# <p align="center"><u> ROS-Brownian-motion-trajectory-generation-and-visualization</u></p>
The mini project involved creating three ROS nodes: p1, p2, and s1. These nodes were designed to collaborate in generating normally distributed samples, selecting, and processing messages, and visualizing the trajectory of a Brownian motion function.
## <p align="center"><u>Introduction and Description</u></p>
In the fourth mini project of EE245: Robotics Programming, we focused on implementing a system of three ROS nodes to simulate Brownian motion trajectory generation and visualization. The primary objective was to explore the message passing capability of ROS through the interaction of publishers and subscribers. 
Overview of the Mini-Project:
The mini project involved creating three ROS nodes: p1, p2, and s1. These nodes were designed to collaborate in generating normally distributed samples, selecting, and processing messages, and visualizing the trajectory of a Brownian motion function.
p1 and p2 Nodes:
The p1 and p2 nodes acted as publishers, responsible for generating normally distributed samples using the Box-Muller transformation. These nodes published the generated samples to ROS topics topic_p1 and topic_p2, respectively, at a fixed rate of 100ms.
s1 Node:
The s1 node served as a subscriber, subscribing to both topic_p1 and topic_p2 to receive messages from the p1 and p2 nodes. It randomly selected one of the message and updated the Brownian motion trajectory based on the selected message, and visualized the trajectory on the console by printing a sequence.
