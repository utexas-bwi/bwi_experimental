#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <fstream>
#include "lib/CameraInterface.h"
#include "lib/SoundInterface.h"
#include "lib/CameraController.h"
#include <std_msgs/Float32.h>

using namespace std;
using namespace ros;

static int current_floor = 3;

/**
 * Configuration struct. Currently, the following have no effect:
 * 
 * area
 * flip
 * strictness
 */
struct config {
    //Ideal HSV values. All values must be out of 255.
    int ideal[3];
    //Range of HSV values. All values must be out of 255.
    int range[3];
    //Min and max area thresholds for triangles found.
    int area[2];
    //Topic to subscribe to.
    string topic;
    //Whether or not to flip the directions each elevator is going.
    bool flip;
    //How strict to be with triangle matching. 0 is the most strict.
    int strictness;
};

//Parse a config file.
void parse(ifstream *, struct config *);

int main(int argc, char **argv) {
    if (argc < 2) {
        cout << "Usage: <config file>" << endl;
        return EXIT_FAILURE;
    }

    ifstream config_file (argv[1]);
    if (!config_file.is_open()) {
        cout << "Config file not found!" << endl;
        return EXIT_FAILURE;
    }

    struct config conf;
    parse(&config_file, &conf);

    init(argc, argv, "elevator_listener");

    NodeHandle node;

    Scalar ideal = Scalar(conf.ideal[0], conf.ideal[1], conf.ideal[2]);
    Scalar range = Scalar(conf.range[0], conf.range[1], conf.range[2]);

    //Servo and controller initialization.
    Publisher cpublisherh = node.advertise<std_msgs::Float32>("/servo0_cmd", 1000);

    CameraController camera(&cpublisherh);
    Subscriber csubscriberh = node.subscribe("/servo0_status", 100, &CameraController::status_callback, &camera);

    //Set area threshold. Parsing problems can't be solved at the moment.
    conf.area[0] = 0;
    conf.area[1] = 1000;

    //Initialize sound interface.
    SoundInterface sinter;

    //Initialize camera interface.
    CameraInterface cinter(ideal, range, &camera, conf.area,
                           conf.flip, conf.strictness, HSV_STANDARD);

    //Print out minimum and maximum Scalar for thresholding.
    cout << cinter.getMin() << endl;
    cout << cinter.getMax() << endl;

    //Subscribe to topics.
    Subscriber listener = node.subscribe("/fft_topic", 100, &SoundInterface::fft_callback, &sinter);
    Subscriber image = node.subscribe(conf.topic, 100, &CameraInterface::image_callback, &cinter);

    //Callbacks.
    spin();

    return 0;
}

void parse(ifstream *file, struct config *configuration) {
    string line;
    bool done = false;
    while (getline(*file, line, '=')) {
        if (line == "image_topic")
            getline(*file, configuration->topic);
        else if (line == "ideal") {
            for (int k = 0; k < 2; k++) {
                getline(*file, line, ' ');
                configuration->ideal[k] = atoi(line.c_str());
            }
            getline(*file, line);
            configuration->ideal[2] = atoi(line.c_str());
        } else if (line == "range") {
            for (int k = 0; k < 2; k++) {
                getline(*file, line, ' ');
                configuration->range[k] = atoi(line.c_str());
            }
            getline(*file, line);
            configuration->range[2] = atoi(line.c_str());
        } else if (line == "area_threshold") {
            getline(*file, line, ' ');
            configuration->area[0] = atoi(line.c_str());
            getline(*file, line);
            configuration->area[1] = atoi(line.c_str());
            if (configuration->area[1] < configuration->area[0]) {
                int t = configuration->area[0];
                configuration->area[0] = configuration->area[1];
                configuration->area[1] = t;
            }
        } else if (line == "strictness") {
            getline(*file, line);
            configuration->strictness = atoi(line.c_str());
        } else if (line == "flip_dir") {
            getline(*file, line);
            configuration->flip = (line == "true");
        }
    }
}
