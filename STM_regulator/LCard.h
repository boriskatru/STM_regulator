#pragma once
#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <string>
#include <chrono>
#include <filesystem>
#include "wait_bh.h"
#include "l502api.h"

#define MAX_MODULES_CNT 3
#define ADC_BUF_SIZE_1 48	//������ ��������� ������ ����� ��� Z ��� �������� ������ � �������� �������
#define ADC_BUF_SIZE_2 16000 //������ �������� ������ ����� ��� XY  ��� ������ ���
#define S_CNT_CRIT_NUM 3
#define RECIVE_COUNT_TIMEOUT 5000


static char serial_list[MAX_MODULES_CNT][L502_SERIAL_SIZE];
const char serial_1[L502_SERIAL_SIZE] = "4T439903";
const char serial_2[L502_SERIAL_SIZE] = "4T439894"; 
static int32_t get_list_res;

/// <summary>
/// ������������ ������, ���������� � �����
/// </summary>
class ADC_Collect {
public:
	int ch_count, s_ch_bufsz;
	vector<double> average;
	vector<vector<double>> input;
	int err_cnt;
	int recv_cnt;
	double *current_data;
	/// <summary>
	/// 
	/// </summary>
	/// <param name="ch_count"> ���������� ������� ���������� � ����� </param>
	/// <param name="ADC_BUF_SIZE"> ������ ������ ������ </param>
	ADC_Collect(int ch_count = 1, int ADC_BUF_SIZE = ADC_BUF_SIZE_1);
	/// <summary>
	/// ��������� ������ � �����
	/// </summary>
	void parse_channels();
	/// <summary>
	/// ���������� ������
	/// </summary>
	/// <param name="count"> ���������� ����������� </param>
	/// <param name="ch"> ����� ������ �����</param>
	/// <returns></returns>
	double Average(int count = 8, int ch = 0);
	/// <summary>
	/// ����� ������� ������ � ��������� ������ (��� �������)
	/// </summary>
	void show();
	/// <summary>
	/// ������ ������ � ����
	/// </summary>
	/// <param name="filename">��� � ���� � ������������ �����</param>
	void print_f(string filename="VAC.dat");
};


class LCard {
	/// <summary>
	/// ������� ������ ADC
	/// </summary>
	double ADC_COLLECT_FREQ = 2000000;
	double ADC_FRAME_FREQ = 0;
	int err = 0;
	int ADC_CHANNEL_COUNT = 16;
	uint32_t* buf;
	int ADC_BUF_SIZE;
	int is_reading = 0;
	/// <summary>
	/// ������� ���������� �� ������� DAC
	/// </summary>
	vector<double> cur_volt;
public:

	/// <summary>
	/// �����������
	/// </summary>
	/// <param name="card_No">����� ����� �� ������� ������� </param>
	/// <param name="ADC_CH_COUNT"> ���������� ������������ ������� �����</param>
	LCard(int card_No = 1, int ADC_CH_COUNT = 1, int ADC_BUF_SIZE = ADC_BUF_SIZE_1);
	~LCard();

	uint32_t count_ADC_data = 0;
	t_l502_hnd hnd;
	char* serial;
	ADC_Collect data;
	uint32_t next_lch;

	
	void SetMode(uint32_t flags);
	void SingleAnalogOut(double data, uint32_t channel = L502_DAC_CH1, uint32_t flags = 0x0001);
	void SingleDigitalOut(uint32_t val, uint32_t mask);
	double AsyncSingleAnalogRead(int channel, double freq = -1, uint32_t tout = 1, uint32_t flags = L502_PROC_FLAGS_VOLT);
	double* AsyncAnalogRead(double freq = -1, uint32_t tout = 1, uint32_t flags = 0);
	double SingleDigitalRead();
	void BackstepZ();
	ADC_Collect AnalogRead(int timeout_ms = 0, int bufsize = ADC_BUF_SIZE_1);
	void StopReadStream();

	void StartReadStream();
};

