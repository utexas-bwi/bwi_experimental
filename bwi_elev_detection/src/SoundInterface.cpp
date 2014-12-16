#include "lib/SoundInterface.h"

double max(double, double);

void SoundInterface::fft_callback(const elevators::fft_col::ConstPtr &msg) {
    vector<double> data = msg->data;
    process_audio(data);

}

void SoundInterface::process_audio(vector<double> data) {
    if (peaks == NULL)
        peaks = (int *)malloc(data.size() * sizeof(int));

    int k = 0;
    for (; k < data.size(); k++) {
        peaks[k] = peak(k, &data);
        //    cout << peaks[k] << " ";
    }
    //  cout << endl;
}

int SoundInterface::peak(int index, vector<double> *dataptr) {
    vector<double> data = *dataptr;
    const int dsize = data.size() - 1;
    if (index == 0)
        return (int)max((data[1] - data[0]) / PEAK_MIN, 0);
    if (index == dsize)
        return (int)max((data[dsize] - data[dsize - 1]) / PEAK_MIN, 0);

    return (int)max(max((data[index] - data[index - 1]) / PEAK_MIN,
                        (data[index + 1] - data[index]) / PEAK_MIN), 0);
}

void SoundInterface::load_ref_data() {
    // vector<float> *v = new vector<float> ();

    // DIR *dir;
    // dirent *ent;
    // string line;
    // if((dir = opendir("")) != NULL){
    //   while((ent = readdir(dir)) != NULL){
    //     ifstream file;
    //     file.open(ent->d_name);

    //     vector<double> processed;

    //     getline(file, line);

    //     if(line != "#elevators/data")
    //       continue;

    //     while(getline (file, line)){

    //     }

    //     file.close();
    //   }
    // } else {
    //   ROS_ERROR("Could not open directory!");
    //   return;
    // }

    // sound_db.push_back(v);
}

double max(double a, double b) {
    return a < b ? b : a;
}
