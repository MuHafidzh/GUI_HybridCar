#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>

#define PORT 9798
#define MAXLINE 64

uint8_t tx_buffer[64];
uint8_t rx_buffer[64];

uint32_t epoch_from_pc = 0;
uint32_t epoch_to_pc;

int sockfd;
struct sockaddr_in servaddr;
socklen_t len;
bool connected = false;

ros::Publisher data_pub;

bool initConnection() {
    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
        perror("socket creation failed");
        return false;
    }

    memset(&servaddr, 0, sizeof(servaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr("192.168.69.2");

    // Set a timeout for the socket
    struct timeval timeout;
    timeout.tv_sec = 1; // 1 second timeout
    timeout.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt failed");
        return false;
    }

    connected = true;
    ROS_INFO("Connected to server");
    return true;
}

bool reconnect() {
    close(sockfd);
    connected = false;
    return initConnection();
}

void timerCallback(const ros::TimerEvent&) {
    if (!connected) {
        if (!reconnect()) {
            ROS_WARN("Reconnection failed, will retry...");
            return;
        }
    }

    memcpy(tx_buffer, &epoch_from_pc, 4);

    if (sendto(sockfd, tx_buffer, sizeof(tx_buffer), MSG_CONFIRM, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("sendto failed");
        connected = false;
        ROS_WARN("Send failed, will retry...");
        return;
    }
    ROS_INFO("Data sent: %d", epoch_from_pc);

    int n = recvfrom(sockfd, rx_buffer, sizeof(rx_buffer), MSG_WAITALL, (struct sockaddr *)&servaddr, &len);
    if(n < 0){
        perror("recvfrom failed");
        connected = false;
        ROS_WARN("Receive failed, will retry...");
        return;
    } 

    memcpy(&epoch_to_pc, rx_buffer, 4);
    ROS_INFO("Data received: %d", epoch_to_pc);      

    epoch_from_pc++;
}

void timer10hzCallback(const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Data received: " << epoch_to_pc;
    msg.data = ss.str();
    data_pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "udp_client");
    ros::NodeHandle nh;

    data_pub = nh.advertise<std_msgs::String>("udp_data", 10);

    if (!initConnection()) {
        ROS_ERROR("Initial connection failed. Exiting...");
        return -1;
    }

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback); // 50Hz timer
    ros::Timer timer10hz = nh.createTimer(ros::Duration(0.1), timer10hzCallback); // 10Hz timer

    ros::spin();

    close(sockfd);
    return 0;
}