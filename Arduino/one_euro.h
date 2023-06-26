#ifndef ONE_EURO_H_
#define ONE_EURO_H_

// Vectores de valores

	
float smoothing_factor(float time_delta, float cutoff);
float exponential_smoothing(float alpha,float x,float x_prev);
void OneEuro_RPY (float* ypr_t,float* ypr_prev_t, float* dypr_prev_t,float* ypr_filt_t, float time_delta, char RPY, float fcmin_n, float beta_n);


#endif