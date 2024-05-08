#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f0xx.h"

#define ON 1
#define OFF 0

class Timer {
private:
	uint32_t timerStartTime;
	uint8_t timerState;
	uint32_t eventTime;
	uint8_t eventFlag;

public:
	static uint32_t timerClassCounter;
	Timer();
	Timer(uint32_t eventTime);
	void start();
	void stop();
	void reset();
	void setEventTime(uint32_t time);
	uint8_t isEvent();
	uint8_t isOn();
};

#endif
