#include "ROSInterface.h"
#include "elevators/fft_col.h"

using namespace ros;
using namespace std;

/**
 * FFT Node, written by Brian Wang, Fall 2014
 * Library by Jivko Sinapov
 *
 * audio_capture must be running
 * Takes input from audio_capture publishes FFT columns.
 */
int main(int argc, char **argv) {
    init(argc, argv, "fft_node");

    bool viz = argc > 1 && (!strcmp("true", argv[1]) || atoi(argv[1]));

    NodeHandle node;

    Publisher press = node.advertise<elevators::fft_col>("fft_topic", 100);
    ROSInterface interface(&press, viz);

    Subscriber feed = node.subscribe("/audio", 100, &ROSInterface::audio_callback, &interface);

    spin();

    return 0;
}


void ROSInterface::audio_callback(
    const audio_common_msgs::AudioData::ConstPtr &msg) {
    vector<uint8_t> data = msg->data;
    perform_fft(data);
}

void ROSInterface::perform_fft(vector<uint8_t> data) {
    int k = 0;
    for (; k < data.size(); k++) {
        float sample = ((float) data[k]) / MAX_AMPLITUDE;
        if (calculator->add_sample_RT(sample))
            publish_fft_col();
    }
}

void ROSInterface::publish_fft_col() {
    vector <double> fft_col = calculator->get_last_fft_column();
    elevators::fft_col msg;
    msg.data = fft_col;

    if (visualizer != NULL){
        visualizer->update(fft_col);
        ROS_INFO("update viz\n");
    }
    out->publish(msg);
}