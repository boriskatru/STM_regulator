#pragma once
#include <stdio.h>
#include <vector>
#include <math.h>
#include <string>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <windows.h>
#include <string>
#include "wait_bh.h"

using namespace std;


#define MAX_MODULES_CNT 3
#define ADC_BUF_SIZE_1 1	//размер короткого буфера платы оси Z для быстрого чтения и обратной реакции
#define ADC_BUF_SIZE_2 2000000 //размер длинного буфера платы оси XY  для чтения ВАХ
#define S_CNT_CRIT_NUM 3
#define RECIVE_COUNT_TIMEOUT 5000



class Card {

public:
	int			PTR_ = 1;	// points to read
	int			PTW_ = 1;	// points to write
	int			ADC_CHANNEL_COUNT = 1;
	
	Timer		tmr;
	long int	read_cnt = 0;

	bool		is_reading = 0;
	bool		is_writing[2] = { 0,0 };
	double		cur_volt[2] = { 0,0 };	// текущее напряжение на выводах DAC

	string		name;
	int			error = 0;
	
	Card(int PTR_ = 1, int ADC_CHANNEL_COUNT = 1, string name = "default", int	PTW_ = 1);
	virtual void SetMode(uint32_t flags) = 0;

	virtual void SingleAnalogOut(double signal, unsigned int channel = 0, double timeout = 0.1) = 0;
	virtual double SingleAnalogRead(int channel = 0, double timeout = 0) = 0;

	
	virtual void SingleDigitalOut(uint32_t val, uint32_t mask) = 0;
	virtual double SingleDigitalRead() = 0;

	virtual void BackstepZ();

	virtual void StopReadStream() = 0;
	virtual void StartReadStream() = 0;
	virtual void FullStop() = 0;

};

