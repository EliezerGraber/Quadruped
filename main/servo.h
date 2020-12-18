#include "driver/ledc.h"

const int _freqHz = 50;

typedef struct Servo{
	ledc_channel_t _ledcChannel;
	unsigned int _min;
	unsigned int _max;
	ledc_timer_bit_t _timerResolution;
};

double servoGetDutyByPercentage(double percentage);

double servoGetDutyByuS(double uS);

void servoAttach(Servo servo, gpio_num_t pin, unsigned int minuS, unsigned int maxuS, ledc_channel_t ledcChannel, ledc_timer_t ledcTimer);

void servoDetach(Servo servo);

void servoWriteMicroSeconds(Servo servo, unsigned int uS);

void servoWrite(Servo servo, unsigned int value);

