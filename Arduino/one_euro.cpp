
#ifndef  F_CPU
	#define F_CPU  16000000UL
#endif


#include <math.h>
#include "stdlib.h"

// Vectores de valores
	
float smoothing_factor(float time_delta_t, float cutoff_t){
	float r = 2*M_PI*cutoff_t*time_delta_t;
	return r/(r+1);
};

float exponential_smoothing(float alpha_t,float x_t,float x_prev_t){
	return alpha_t*x_t + (1 - alpha_t)*x_prev_t;
};

void OneEuro_RPY (float* ypr_t,float* ypr_prev_t, float* dypr_prev_t,float* ypr_filt_t, float time_delta, char RPY, float fcmin_n, float beta_n ){
  
  // Variables para filtro 1 Euro
  float cutoff, alpha, alpha_d, dx ,dx_hat, x_hat;
  // Variables previas

  char size = 4;
  if (RPY == 1)
    size = 2;

	for (char i = 0; i < size; i++){
		
		// Filtro 1 Euro
		alpha_d  = smoothing_factor(time_delta, fcmin_n);
		dx = (ypr_t[i] - ypr_prev_t[i]) / time_delta;
		dx_hat = exponential_smoothing(alpha_d, dx, dypr_prev_t[i]);
		
		// Filtro de la se�al
		cutoff = fcmin_n + beta_n*abs(dx_hat);
		alpha = smoothing_factor(time_delta, cutoff);
		x_hat = exponential_smoothing(alpha, ypr_t[i], ypr_prev_t[i]);
		
		if (isnan(x_hat)){
			if (isnan(ypr_t[i]))
				ypr_filt_t[i] = 0;
			else
				ypr_filt_t[i] = ypr_t[i];
		}
		else{
			ypr_filt_t[i] = x_hat;			
		}
		

		ypr_prev_t[i] = x_hat;
		dypr_prev_t[i] = dx_hat;
	}


}
