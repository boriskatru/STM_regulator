#pragma once
#include <string>
#include <stdio.h>

///////////////////// НАСТРОЙКИ LCARD /////////////////////

#define MAX_MODULES_CNT 3
#define ADC_CHANEL_CNT 6
#define ADC_BUF_SIZE_1 1		
#define ADC_BUF_SIZE_2 4000000			// размер длинного буфера платы оси XY  для чтения ВАХ
#define ADC_BUF_SIZE_3 300000  //размер длинного буфера платы оси XY  для калибровок 
#define S_CNT_CRIT_NUM 3
#define RECIVE_COUNT_TIMEOUT 5000
#define ANALOG_OUT_FLAG 0x0001
#define ADC_TGT_FREQ 2000000

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


////////////// Настройки FINE движения для PID ///////////////
#define Z_COARSE_PREC 0.05				//диапазон fine регуляции без изменения сигнала coarse
#define Z_OUT 1U						//номер канала coarse
#define Z_OUT_FINE 0U					//номер канала fine



///////////////////// ОБЩИЕ НАСТРОЙКИ РЕГУЛЯТОРА /////////////////////

#define MIN_STEP_SIZE 0.00015258789			// Минимальный размер шага напряжения ЦАП (примерно 0.55 ангстрем в COARSE и 0.035 ангстрем в FINE)
#define I_to_nA 10
#define FORWARD true
#define BACKWARD false


/////////////////////  НАСТРОЙИ ПЬЕЗОПОДВИЖЕК //////////////
#define X_FW_BW 0.9492						// (значение Бориса 0.852) отношения шагоа вперёд к шагу назад по оси X на комнате
#define Y_FW_BW 1							// 0.881  отношения шагоа вперёд к шагу назад по оси Y на комнате


#define X_PLANE_TG 0.03						// Коэффициент наклона при расширении пьезоподвижек (отношение смещения по Z к смещению по оси X)
#define Y_PLANE_TG 0.02						// Коэффициент наклона при расширении пьезоподвижек (отношение смещения по Z к смещению по оси Y)


#define COARSE_TO_FINE 14.6

#define DEFAULT_MICROSTEP_SIZE 0.00001

///////////////////// дефолтные коэффициенты PID регуляции //////////////////////
#define Pc 0.00003
#define Ic 0.0000024
#define Dc 0


///////////////////// Пути и названия файлов настроек и связи с интерфесом //////////////////////
const std::string MAIN_FOLDER = "C:/Users/Tunnel Noise/Desktop/STM/";

const std::string SCAN_FOLDER = "scans/";

const std::string SETTINGS_FOLDER = "Settings/";

const std::string PID_SETTINGS_FILE = SETTINGS_FOLDER + "PID_SETTINGS.txt";

const std::string EXEC_STATUS_FILE = SETTINGS_FOLDER + "status/EXEC_STATUS.txt";

const std::string COMMAND_STATUS_FILE = SETTINGS_FOLDER + "status/COMMAND_STATUS.txt";

const std::string PROGRESS_STATUS_FILE = SETTINGS_FOLDER + "status/PROGRESS_STATUS.txt";


const std::string EQUIPMENT_STATUS_FILE = SETTINGS_FOLDER + "EQUIPMENT_STATUS.txt";

const std::string StepXY_SETTINGS = SETTINGS_FOLDER + "StepXY.txt";
const std::string MoveXY_SETTINGS = SETTINGS_FOLDER + "MoveXY.txt";
const std::string Landing_SETTINGS = SETTINGS_FOLDER + "Landing.txt";
const std::string VANC_SETTINGS = SETTINGS_FOLDER + "VANC.txt";
const std::string CapStepScan_SETTINGS = SETTINGS_FOLDER + "CapStepScan.txt";
const std::string TouchScan_SETTINGS = SETTINGS_FOLDER + "TouchScan.txt";
const std::string Retract_SETTINGS = SETTINGS_FOLDER + "Retract.txt";
const std::string PNCalibr_SETTINGS = SETTINGS_FOLDER + "Pn_CVg_TransistorCalibration.txt";
const std::string RCalibr_SETTINGS = SETTINGS_FOLDER + "R_CVg_TransistorCalibration.txt";

const std::string SESSION_FILE_NAME = SETTINGS_FOLDER + "SESSION_DATA.txt";

const std::string VANC_LIST_NAME = "vac_list.txt";
const std::string SCAN_LIST_NAME = "scans_list.txt";
const std::string CAP_SCAN_LIST_NAME = "cap_scans_list.txt";