#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <alsa/asoundlib.h>

#include "alsa_utils.h"

#include "fft_calculator_dq.h"
#include "fft_visualizer.h"

#include "BackgroundModelFFT.h"

	      
const static int SAMPLE_RATE = 44100; 
const static int BUFF_SIZE = 128;
	      
#define NFFT 128	//number of frequency bins in FFT, must be a power of 2
#define FFT_WIN 128	//length of window used to compute FFT (in terms of number of raw signal samples)
#define NOVERLAP 64	//by how much the windows should overlap - typically, half of FFT_WIN

bool g_caught_sigint = false;

snd_pcm_t *capture_handle;

int i,j;
int err;
short buf[128];

BackgroundModelFFT* BGM;

void sig_handler(int sig)
{
	g_caught_sigint = true;
	snd_pcm_close (capture_handle);
	exit (0);
};

vector< vector<double> > record_fft_columns(double duration, FFT_calculator* fft_calc){
	int num_samples_to_record = (int)((double)SAMPLE_RATE*duration);
	int c = 0;
	int num_fft_columns = 0;
	int err;
	
	vector< vector<double> > result;
	
	while (true){
		if ((err = snd_pcm_readi (capture_handle, buf, BUFF_SIZE)) != BUFF_SIZE) {
			fprintf (stderr, "read from audio interface failed (%s)\n",snd_strerror (err));
			exit (1);
		}
		else {
				for (j = 0; j < BUFF_SIZE; j++){	
					float sample_j = (float)buf[j]/(float)32768.0;
		
					if (fft_calc->add_sample_RT(sample_j)){
						vector<double> next_fft = fft_calc->get_last_fft_column();
						result.push_back(next_fft);
						num_fft_columns++;
					}
					
					c++;
					
				}
		}
		
		if (c > num_samples_to_record)
			break;
	}
	
	printf("Listened for %i samples and %i fft columns...\n",c,num_fft_columns);
	return result;
}
	

void learn_sound_model(vector< vector<double> > fft_columns){
	//step 1: assume the first 0.5 seconds are background and use that to learn background model
	int t = (int)((double)(SAMPLE_RATE / 2)/(double)NOVERLAP);
	
	vector< vector<double> > background_fft_columns ;
	
	for (i = 0; i < t; i ++){
		background_fft_columns.push_back(fft_columns.at(i));
	}
	
	BGM = new BackgroundModelFFT();
	BGM->estimateModel(background_fft_columns);
	
}	
	      
main (int argc, char *argv[])
	{
		//open a handle to the microphone
		capture_handle = open_mic_capture("hw:2,0",SAMPLE_RATE);

		//visualizer for FFT
		FFT_visualizer *FFT_viz;
		int viz_range = 640; //show last 640 samples of the FFT
		bool visualize = true;

		/* INITIALIZE FFT */
		FFT_calculator* FFT = new FFT_calculator(NFFT,FFT_WIN,NOVERLAP);
		FFT->init_RT_mode();

		//read half a second from the mic, this is needed upon start
		vector< vector<double> > temp = record_fft_columns(0.5,FFT);
		printf("Read %i fft columns...\n",(int)temp.size());
		temp.clear();


		if (argc > 1){
			printf("Training mode. Enter the duration to listen for and press 'Enter'\n");
			double d;
			std::cin >> d;
			sleep(2);
			temp = record_fft_columns(d,FFT);
			
			learn_sound_model(temp);
		}


		//read from the microphone
		
		
		
		/*for (i = 0; i < 1000; ++i) {
		//while (true){
			
			if ((err = snd_pcm_readi (capture_handle, buf, BUFF_SIZE)) != BUFF_SIZE) {
				fprintf (stderr, "read from audio interface failed (%s)\n",snd_strerror (err));
				exit (1);
			}
			else {
				for (j = 0; j < BUFF_SIZE; j++){
					//FFT->add_sample_RT((float)buf[j]);
					
					float sample_j = (float)buf[j]/(float)32768.0;
	
					//printf("%i\n",buf[j]);
					if (FFT->add_sample_RT(sample_j)){
						//get last sample
						vector<double> next_fft = FFT->get_last_fft_column();
				
					}
				}
			}
		}*/
	
		snd_pcm_close (capture_handle);
		exit (0);
	}
