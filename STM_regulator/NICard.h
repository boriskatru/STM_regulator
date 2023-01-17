#pragma once
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <iostream>
#include <windows.h>
#include <NIDAQmx.h>  
#include "wait_bh.h"
#include "Card.h"


#define NI_BUF_SIZE 1
inline double ZERO_ARR[2] = { 0,0 };

int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
int32 CVICALLBACK EveryNCallback2(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData);
//#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

using namespace std;


class NICard:public Card {
public:
	
	char        errBuff[2048] = { '\0' };
	bool		autostart = 0;

	TaskHandle   ai_0 ;
	TaskHandle   ao_0 ;
	TaskHandle   ao_1 ;

	double*	 data_;


	NICard();
	~NICard();

	void SetMode(uint32_t flags) ;

	void SingleAnalogOut(double signal, unsigned int channel = 1, double timeout = 0.1);
	void SingleAnalogOut(double* signal);
	double SingleAnalogRead(int channel = 0, double timeout = 0);
	
	void SingleDigitalOut(uint32_t val, uint32_t mask) ;
	double SingleDigitalRead();

	void StopDAC(int channel = 0);
	void StartDAC(int channel = 0);

	void StopReadStream() ;
	void StartReadStream() ;

	void FullStop();
};

