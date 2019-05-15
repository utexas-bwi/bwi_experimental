#ifndef AUDIO_INTERFACE_H
#define AUDIO_INTERFACE_H

#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include "fft_calculator_dq.h"
#include "fft_visualizer.h"
#include <audio_common_msgs/AudioData.h>

//Maximum amplitude of samples
#define MAX_AMPLITUDE 255
//Graph window, in FFT columns (for visualization)
#define GRAPH_WINDOW 1024

using namespace std;
using namespace ros;

class ROSInterface {
    Publisher *out;
    FFT_calculator *calculator;
    FFT_visualizer *visualizer;
public:
    ROSInterface(Publisher *p, bool viz) {
        out = p;
        calculator = new FFT_calculator();
        visualizer = viz ?
                     new FFT_visualizer(calculator->get_num_fft(), GRAPH_WINDOW) :
                     NULL;
        calculator->init_RT_mode();
    }
    ~ROSInterface() {
        delete calculator;
        if (visualizer != NULL)
            delete visualizer;
    }
    void audio_callback(
        const audio_common_msgs::AudioData::ConstPtr &msg);
private:
    void perform_fft(vector<uint8_t> data);
    void publish_fft_col();
};

#endif //AUDIO_INTERFACE_H