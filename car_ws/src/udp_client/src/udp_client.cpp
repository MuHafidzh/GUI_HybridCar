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

int16_t RpmInput, PhaseCurrentInput;
uint8_t ControlCmd, direction;

float RealVolt, RealCurrent, RealPhaseCurrent; //satuan volt dan Ampere
int16_t Speed;
uint16_t adc_value;

typedef struct {
    int8_t controlTemperature; // byte 0
    int8_t motorTemperature;   // byte 1
    uint8_t status;            // byte 2
    uint8_t errorCode[3];      // byte 3-5
    uint8_t reserved;          // byte 6
    uint8_t liveSignal;        // byte 7

    // New variables to store the status of each bit
    uint8_t isRunning;
    uint8_t controlMode;
    uint8_t overcurrent;
    uint8_t overload;
    uint8_t overvoltage;
    uint8_t undervoltage;
    uint8_t controllerOverheat;
    uint8_t motorOverheat;
    uint8_t motorStalled;
    uint8_t motorOutOfPhase;
    uint8_t motorSensor;
    uint8_t motorAuxSensor;
    uint8_t encoderMisaligned;
    uint8_t antiRunawayEngaged;
    uint8_t mainAccelerator;
    uint8_t auxAccelerator;
    uint8_t preCharge;
    uint8_t dcContactor;
    uint8_t powerValve;
    uint8_t currentSensor;
    uint8_t autoTune;
    uint8_t rs485;
    uint8_t can;
    uint8_t software;
} MotorStatus;

MotorStatus motorData;

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
    /*		memcpy(&PC_Epoch, pc_recv, 4);
    memcpy(&PhaseCurrentInput, pc_recv+4, 2);
    memcpy(&RpmInput, pc_recv+6, 2);
    memcpy(&ControlCmd, pc_recv+8, 1);
    memcpy(&direction, pc_recv+9, 1);*/
    memcpy(tx_buffer, &epoch_from_pc, 4);
    memcpy(tx_buffer+4, &PhaseCurrentInput, 2);
    memcpy(tx_buffer+6, &RpmInput, 2);
    memcpy(tx_buffer+8, &ControlCmd, 1);
    memcpy(tx_buffer+9, &direction, 1);

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
    /*	memcpy(pc_send+0, &epoch_kirim, 4);
    memcpy(pc_send+4, &RealVolt, 4);
    memcpy(pc_send+8, &RealCurrent, 4);
    memcpy(pc_send+12, &RealPhaseCurrent, 4);
    memcpy(pc_send+16, &Speed, 4);
    memcpy(pc_send+20, &receivedData.controlTemperature, 1);
    memcpy(pc_send+21, &receivedData.motorTemperature, 1);
    memcpy(pc_send+22, &receivedData.status, 1);
    memcpy(pc_send+23, &receivedData.errorCode[0], 1);
    memcpy(pc_send+24, &receivedData.errorCode[1], 1);
    memcpy(pc_send+25, &receivedData.errorCode[2], 1);
    memcpy(pc_send+26, &receivedData.reserved, 1);
    memcpy(pc_send+27, &receivedData.liveSignal, 1);
    memcpy(pc_send+28, &adc_value, 2); //throttle
    //nnti encoder sama proximity sensor   */
    memcpy(&epoch_to_pc, rx_buffer, 4);
    memcpy(&RealVolt, rx_buffer+4, 4);
    memcpy(&RealCurrent, rx_buffer+8, 4);
    memcpy(&RealPhaseCurrent, rx_buffer+12, 4);
    memcpy(&Speed, rx_buffer+16, 4);
    memcpy(&motorData.controlTemperature, rx_buffer+20, 1);
    memcpy(&motorData.motorTemperature, rx_buffer+21, 1);
    memcpy(&motorData.status, rx_buffer+22, 1);
    memcpy(&motorData.errorCode[0], rx_buffer+23, 1);
    memcpy(&motorData.errorCode[1], rx_buffer+24, 1);
    memcpy(&motorData.errorCode[2], rx_buffer+25, 1);
    memcpy(&motorData.reserved, rx_buffer+26, 1);
    memcpy(&motorData.liveSignal, rx_buffer+27, 1);
    memcpy(&adc_value, rx_buffer+28, 2);

    

    ROS_INFO("Data received: %d", epoch_to_pc);      

    epoch_from_pc++;
}

void timer10hzCallback(const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Data received: " << epoch_to_pc << "\n";
    ss << "Throttle: " << adc_value << "\n";
    msg.data = ss.str();
    data_pub.publish(msg);
}

void Process_Received_Data(MotorStatus *data) {
    data->isRunning = (data->status & (1 << 0)) ? 1 : 0;
    data->controlMode = (data->status & (1 << 1)) ? 1 : 0;

    data->overcurrent = (data->errorCode[0] & (1 << 0)) ? 1 : 0;
    data->overload = (data->errorCode[0] & (1 << 1)) ? 1 : 0;
    data->overvoltage = (data->errorCode[0] & (1 << 2)) ? 1 : 0;
    data->undervoltage = (data->errorCode[0] & (1 << 3)) ? 1 : 0;
    data->controllerOverheat = (data->errorCode[0] & (1 << 4)) ? 1 : 0;
    data->motorOverheat = (data->errorCode[0] & (1 << 5)) ? 1 : 0;
    data->motorStalled = (data->errorCode[0] & (1 << 6)) ? 1 : 0;
    data->motorOutOfPhase = (data->errorCode[0] & (1 << 7)) ? 1 : 0;

    data->motorSensor = (data->errorCode[1] & (1 << 0)) ? 1 : 0;
    data->motorAuxSensor = (data->errorCode[1] & (1 << 1)) ? 1 : 0;
    data->encoderMisaligned = (data->errorCode[1] & (1 << 2)) ? 1 : 0;
    data->antiRunawayEngaged = (data->errorCode[1] & (1 << 3)) ? 1 : 0;
    data->mainAccelerator = (data->errorCode[1] & (1 << 4)) ? 1 : 0;
    data->auxAccelerator = (data->errorCode[1] & (1 << 5)) ? 1 : 0;
    data->preCharge = (data->errorCode[1] & (1 << 6)) ? 1 : 0;
    data->dcContactor = (data->errorCode[1] & (1 << 7)) ? 1 : 0;

    data->powerValve = (data->errorCode[2] & (1 << 0)) ? 1 : 0;
    data->currentSensor = (data->errorCode[2] & (1 << 1)) ? 1 : 0;
    data->autoTune = (data->errorCode[2] & (1 << 2)) ? 1 : 0;
    data->rs485 = (data->errorCode[2] & (1 << 3)) ? 1 : 0;
    data->can = (data->errorCode[2] & (1 << 4)) ? 1 : 0;
    data->software = (data->errorCode[2] & (1 << 5)) ? 1 : 0;
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