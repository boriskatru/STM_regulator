#pragma once

///////////////////// НАСТРОЙКИ LCARD /////////////////////
#define MAX_MODULES_CNT 3
#define ADC_BUF_SIZE_1 1		
#define ADC_BUF_SIZE_2 4000000			// размер длинного буфера платы оси XY  для чтения ВАХ
#define S_CNT_CRIT_NUM 3
#define RECIVE_COUNT_TIMEOUT 5000

////////////// Номера и назначения каналов АЦП ///////////////
#define BIAS_CH 0
#define CURRENT_CH 4
#define NOISE_CH 2
#define R_CALIBR_CH 4

////////////// Номера и назначения каналов ЦАП ///////////////
#define BIAS_OUT L502_DAC_CH2
#define X_OUT L502_DAC_CH1
#define Y_OUT L502_DAC_CH2
#define X_ L502_DAC_CH1
#define Y_ L502_DAC_CH2

///////////////////// НАСТРОЙКИ ПЛАТЫ NI /////////////////////

#define NI_BUF_SIZE 1					//размер короткого буфера платы оси Z для быстрого чтения и обратной реакции

#define ANALOG_OUT_FLAG 0x0001
#define ADC_TGT_FREQ 2000000
#define FORWARD true
#define BACKWARD false

////////////// Настройки FINE движения для PID ///////////////
#define Z_COARSE_PREC 0.05				//диапазон fine регуляции без изменения сигнала coarse
#define Z_OUT 1U						//номер канала coarse
#define Z_OUT_FINE 0U					//номер канала fine



///////////////////// ООБЩИЕ НАСТРОЙКИ РЕГУЛЯТОРА /////////////////////

#define MIN_STEP_SIZE 0.00015258789			// Минимальный размер шага напряжения ЦАП (примерно 0.55 ангстрем в COARSE и 0.035 ангстрем в FINE)

/////////////////////  соотношение шагов пьезоподвижек вперёд и назад //////////////
#define X_FW_BW 0.9492						// (значение Бориса 0.852) отношения шагоа вперёд к шагу назад по оси X на комнате
#define Y_FW_BW 1							// 0.881  отношения шагоа вперёд к шагу назад по оси Y на комнате



///////////////////// коэффициенты PID регуляции //////////////////////
///#define Pc 0.0001
#define Pc 0.00003
//0.000000003 Boris original
// 0.03 my value ( very stable)
#define Ic 0.0000024
#define Dc 0






