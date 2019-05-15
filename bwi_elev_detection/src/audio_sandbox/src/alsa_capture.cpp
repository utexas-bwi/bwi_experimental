#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>

#include "alsa_utils.h"

#include "fft_calculator_dq.h"
#include "fft_visualizer.h"

	      
const static int SAMPLE_RATE = 44100; 
const static int BUFF_SIZE = 128;
	      
#define NFFT 128	//number of frequency bins in FFT, must be a power of 2
#define FFT_WIN 128	//length of window used to compute FFT (in terms of number of raw signal samples)
#define NOVERLAP 64	//by how much the windows should overlap - typically, half of FFT_WIN

bool g_caught_sigint = false;

snd_pcm_t *capture_handle;


void sig_handler(int sig)
{
	g_caught_sigint = true;
	snd_pcm_close (capture_handle);
	exit (0);
};
	      
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

		//read from the microphone
		int i,j;
		int err;
		short buf[128];
		
		for (i = 0; i < 1000; ++i) {
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
						
						for (int k =0; k < next_fft.size(); k++){
							printf("%f,",next_fft.at(k));
						}
						printf("\n");
					}
				}
			}
		}
	
		snd_pcm_close (capture_handle);
		exit (0);
	}
