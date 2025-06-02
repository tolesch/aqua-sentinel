#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <ESP32Servo.h>

// Servo setup
Servo servo1, servo2;
const int servo_pin1 = 18;
const int servo_pin2 = 26;

// Micro-ROS variables
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Vector3 sub_msg;

// UDP discovery setup
WiFiUDP udp;
const int DISCOVERY_PORT = 9999;
const char* DISCOVERY_MSG = "WHERE_IS_MY_ROBOT_OVERLORD";
const uint16_t agent_port = 8888;

// Callback for subscription
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
  Serial.printf("Received: x=%.2f, y=%.2f, z=%.2f\n", msg->x, msg->y, msg->z);
  float pos1 = constrain(msg->x, 0, 180);
  float pos2 = constrain(msg->y, 0, 180);
  servo1.write(pos1);
  servo2.write(pos2);
  
  // // Control pump speed based on z (0.0 to 1.0)
  // int pump_pwm = (int)(msg->z * 255);
  // analogWrite(pump_pin, pump_pwm);
  // Control pump - digital on/off based on z value
  if (msg->z > 0.5) {
    digitalWrite(pump_pin, HIGH); // Full power ON
    Serial.println("Pump ON");
  } else {
    digitalWrite(pump_pin, LOW); // OFF
    Serial.println("Pump OFF");
  }
}

// Function to discover Micro-ROS agent IP
IPAddress discover_agent_ip() {
  udp.begin(DISCOVERY_PORT);
  udp.beginPacket("255.255.255.255", DISCOVERY_PORT);
  udp.write((uint8_t*)DISCOVERY_MSG, strlen(DISCOVERY_MSG));
  udp.endPacket();

  unsigned long start_time = millis();
  while (millis() - start_time < 5000) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char packetBuffer[255];
      int len = udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
        Serial.println("Received response: " + String(packetBuffer));
        IPAddress agent_ip;
        if (agent_ip.fromString(packetBuffer)) {
          Serial.println("Valid agent IP: " + String(packetBuffer));
          return agent_ip;
        } else {
          Serial.println("Invalid IP format in response");
        }
      }
    }
    delay(100);
  }
  Serial.println("No response received within timeout");
  return IPAddress(0, 0, 0, 0);
}

void setup() {
  Serial.begin(115200);
  servo1.attach(servo_pin1);
  servo2.attach(servo_pin2);
  
  // Connect to WiFi using WiFiManager
  delay(1000);
  WiFiManager wm;
  Serial.println("Starting WiFiManager...");
  if (!wm.autoConnect("ESP32_Config_AP")) {
    Serial.println("Failed to connect to WiFi, restarting...");
    ESP.restart();
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Loop until Micro-ROS agent is found and initialized
  while (true) {
    IPAddress agent_ip = discover_agent_ip();
    if (agent_ip == IPAddress(0, 0, 0, 0)) {
        Serial.println("Agent not found, retrying in 5 seconds...");
        delay(5000);
        continue;
    }
    Serial.print("Received answer from agent at: ");
    Serial.println(agent_ip);
    // Set up Micro-ROS with discovered agent IP
    set_microros_wifi_transports(NULL, NULL, agent_ip, agent_port);
    if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
        Serial.println("Micro-ROS support initialized");
        break; // Exit loop on successful initialization
    } else {
        Serial.println("Failed to initialize Micro-ROS, retrying...");
        delay(5000);
    }
  }
  // Initialize Micro-ROS node
  if (rclc_node_init_default(&node, "micro_ros_node", "", &support) != RCL_RET_OK) {
    Serial.println("Failed to initialize node");
    ESP.restart();
  }
  // Initialize subscriber
  if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "servo_positions") != RCL_RET_OK) {
    Serial.println("Failed to initialize subscriber");
    ESP.restart();
  }
  // Initialize message and executor
  geometry_msgs__msg__Vector3__init(&sub_msg);
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to initialize executor");
    ESP.restart();
  }
  if (rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add subscription");
    ESP.restart();
  }
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
    delay(1000);
    return;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));  // Spin with 10 ms timeout
  delay(1);  // Small delay to yield CPU
}