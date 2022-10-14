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
#include "Card.h"

#define ANALOG_OUT_FLAG 0x0001


static char serial_list[MAX_MODULES_CNT][L502_SERIAL_SIZE];
const char serial_1[L502_SERIAL_SIZE] = "4T439903";
const char serial_2[L502_SERIAL_SIZE] = "4T439894"; 
static int32_t get_list_res;

/// <summary>
/// Обработанные данные, полученных с платы
/// </summary>
class ADC_Collect {
public:
	int ch_count, s_ch_bufsz;
	vector<double> average;
	vector<vector<double>> input;
	int err_cnt = 0;
	int recv_cnt = 0;
	double *current_data;
	/// <summary>
	/// 
	/// </summary>
	/// <param name="ch_count"> Количество каналов считывания с платы </param>
	/// <param name="ADC_BUF_SIZE"> Размер буфера чтения </param>
	ADC_Collect(int ch_count = 1, int ADC_BUF_SIZE = ADC_BUF_SIZE_1);
	/// <summary>
	/// Обработка данных с карты
	/// </summary>
	void parse_channels();
	/// <summary>
	/// Усреднение данных
	/// </summary>
	/// <param name="count"> количество усредняемых </param>
	/// <param name="ch"> номер канала платы</param>
	/// <returns></returns>
	double Average(int count = 8, int ch = 0);
	/// <summary>
	/// Вывод текущих данных в командную строку (для отладки)
	/// </summary>
	void show();
	/// <summary>
	/// Запись данных в файл
	/// </summary>
	/// <param name="filename">имя и путь к создаваемому файлу</param>
	void print_f(string filename="VAC.dat", string directory = "../../scans");
};


class LCard: public Card {
	/// <summary>
	/// Частота опроса ADC
	/// </summary>
	double ADC_COLLECT_FREQ = 2000000;
	double ADC_FRAME_FREQ = 0;
	uint32_t* buf;

public:

	/// <summary>
	/// Конструктор
	/// </summary>
	/// <param name="card_No">номер платы по порядку запуска </param>
	/// <param name="ADC_CH_COUNT"> количество используемых каналов ввода</param>
	LCard(int card_No = 1, int ADC_CH_COUNT = 1, int ADC_BUF_SIZE = ADC_BUF_SIZE_2);
	~LCard();

	uint32_t count_ADC_data = 0;
	t_l502_hnd hnd;
	char* serial;
	ADC_Collect data;
	uint32_t next_lch;
	
	
	void SetMode(uint32_t flags);
	void SingleAnalogOut(double data, unsigned int channel = L502_DAC_CH1, double timeout = 0.1);
	void SingleDigitalOut(uint32_t val, uint32_t mask);
	double SingleAnalogRead(int channel, double freq = -1, uint32_t tout = 1, uint32_t flags = L502_PROC_FLAGS_VOLT); 
	double SingleAnalogRead(int channel = 0, double timeout = 0);
	double* AsyncAnalogRead(double freq = -1, uint32_t tout = 1, uint32_t flags = 0);
	double SingleDigitalRead();
	ADC_Collect AnalogRead(int timeout_ms = 50, int bufsize = ADC_BUF_SIZE_2);
	void StopReadStream();
	void StartReadStream();
	void FullStop();
};

