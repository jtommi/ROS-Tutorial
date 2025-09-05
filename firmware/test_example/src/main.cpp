#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 blink_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const rosidl_message_type_support_t *type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

const uint BTN_PIN = 4;
const uint LED_PIN = 22;
auto last_button_time = millis();

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

// Error handle loop
void error_loop() {
  while (1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  // if (timer != NULL) {
  //   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  //   msg.data++;
  // }
}

void subscription_callback(const void *msgin) {
  // Cast received message to used type
  const std_msgs__msg__Int32 *blink_msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", blink_msg->data);
  for (int i = 0; i < blink_msg->data; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Set pin modes as input
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "blink"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer,
                                  &support,
                                  RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor,
                                         &subscriber,
                                         &blink_msg,
                                         &subscription_callback,
                                         ON_NEW_DATA));

  msg.data = 0;
}

void loop() {
  if (!digitalRead(BTN_PIN)) {
    auto new_time = millis();
    if (new_time - last_button_time > 300) {
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
      msg.data++;
      last_button_time = new_time;
    }
  }
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}