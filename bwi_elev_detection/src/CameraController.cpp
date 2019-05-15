#include "lib/CameraController.h"

void CameraController::moveTo(float f) {
    std_msgs::Float32 msg;
    msg.data = f;
    
    publisher->publish(msg);
}

void CameraController::wait(int ms) {
    usleep(1000 * ms);
}

void CameraController::alternate() {
    if (currentPosition >= 0)
        moveTo(range[0]);
    else
        moveTo(range[1]);
}

void CameraController::status_callback(const std_msgs::Float32::ConstPtr &msg) {
    currentPosition = msg->data;
    if (currentPosition <= range[0] + 0.05 || currentPosition >= range[1] - 0.05)
        counter++;
    if (counter >= 60) {
        counter = 0;
        alternate();
    }
}
