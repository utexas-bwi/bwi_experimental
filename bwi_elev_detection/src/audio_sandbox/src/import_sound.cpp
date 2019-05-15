#include <cstdlib>
#include <string>
#include <iostream>
#include <cmath>
#include "fft_calculator_dq.h"
#include "fft_visualizer.h"

#define NOISE 0.01

int main(int argc, char **argv) {
    if (argc < 3) {
        cout << "Usage: import_sound <infile> <outfile>" << endl;
        return;
    }

    ifstream infile;
    ofstream outfile;
    string line;
    FFT_calculator calc;
    fft_visualizer viz;

    infile.open(argv[1]);
    if (!infile.is_open()) {
        cout << "Unable to open infile!" << endl;
        return EXIT_FAILURE;
    }

    outfile.open(argv[2])
    if (!outfile.is_open()) {
        cout << "Unable to open outfile!" << endl;
        return EXIT_FAILURE;
    }

    vector<float> samples;
    double least = 1.1;

    while (getline(file, line)) {
        double ldata = atof(line);
        samples.push_back((float)ldata);
        least = ldata < least ? ldata : least;
    }

    //TODO: normalize all values, and save FFT

    infile.close();

    double avg = 0;
    int k = 0;
    for (; k < samples.size(); k++) {
        if (samples[k] <= least + NOISE)
            samples[k] = 0;
        else
            avg += samples[k];
    }

    for (k = 0; k < samples.size(); k++){
        samples[k] += -avg + abs(least);
        if(samples[k] > 1)
        	samples[k] = 1;
    }
    calc.compute_fft(samples);

    deque<vector<float>> out_data = calc.get_data();
    for(k = 0; k < out_data.size(); k++){
    	vector<float> out_line = out_data[k];
    	int j = 0;
    	for(; j < out_line.size(); j++)
    		outfile << j << " ";
    	outfile << endl;
    }

    return EXIT_SUCCESS;
}