#pragma once
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <l502api.h>
#include <filesystem>
#include <cstdarg>

#include "piezo_positioners.h"
#include "LCard.h"
#include "vecters.h"
#include "wait_bh.h"
#include "Scans.h"
#include "gwyfile.h"
#include "NICard.h"
#include "ZurichMFLI.h"
#include "Settings.h"

using namespace std;


inline double W_Lambert_approx(double x) {
	double lnxpp = log(x + 1);
	return (0.665 * (1 + 0.0195 * lnxpp) * lnxpp + 0.04);
}
inline double CHTransform(double current, double voltage, double offset = 0.005, double c1 = 2, double c2 = 10) {
	return (W_Lambert_approx(c1 * voltage / max(current, offset)) * c2)* 1.05 * exp(-0.02 / c1 * current);
}
//inline double CHTransform(double current, double voltage, double offset = 0.005, double c1 = 2, double c2 = 10) {
//	return current;
//}
inline double LimCatch(double signal, double limit, double max = 25) {
	if (signal > limit) return signal;
	else return max;
}
inline string get_time_string() {
	time_t rawtime;
	struct tm* timeinfo;
	char buffern[20];								// строка, в которой будет храниться текущая дата
	time(&rawtime);									// текущая дата в секундах
#pragma warning(suppress : 4996)
	timeinfo = localtime(&rawtime);					// текущее локальное время, представленное в структуре
	strftime(buffern, 20, "%x", timeinfo);
	string time = "";
	time += buffern;
	time += "/";
	strftime(buffern, 20, "%H_%M", timeinfo);

	time += buffern;
	return time;
}

inline void make_logs( string text, string folder= MAIN_FOLDER) {
	std::cout << endl << text << endl;
	string datestr = "";
	string timestr = "";

	time_t rawtime;
	struct tm* timeinfo;
	char buffern[20];								// строка, в которой будет храниться текущая дата
	time(&rawtime);									// текущая дата в секундах
#pragma warning(suppress : 4996)
	timeinfo = localtime(&rawtime);					// текущее локальное время, представленное в структуре

	strftime(buffern, 20, "%x", timeinfo);	
	datestr += buffern;
	
	strftime(buffern, 20, "%H_%M", timeinfo);
	timestr += buffern;
	
	std::filesystem::create_directories(folder + SCAN_FOLDER + datestr);
	ofstream file;
	file.open(folder + SCAN_FOLDER + datestr + "/" + "Logs.txt" , std::ios::app);
	file << endl << timestr<< ":" << endl << text << endl;;
	file.close();
}
inline double TrBiasStepper( double Vg, bool dir ) {
	short int sign = 1;
	if (dir == 0) sign = -1;
	double offset = 0.072;
	double ranges[6][2] = {
		{ 0.446, 0.4 },
		{ 0.432, 0.1 },
		{ 0.422, 0.03 },
		{ 0.414, 48 * MIN_STEP_SIZE },
		{ 0.408, 12 * MIN_STEP_SIZE },
		{ 0.400, 4 * MIN_STEP_SIZE }
	};
	for (int i = 0; i < 6; i++ ) {
		if (Vg > ranges[i][0] + offset) return (sign * ranges[i][1]);
	}
	return 2 * sign * MIN_STEP_SIZE;
	
	
}	
 


class PID {
	
public:
	double P;
	double I;
	double D;
	double integral;
	double err ; // невязка
	double d_err;
	double last_signal;
	double tmp;
	int diff_count;
	double bias;
	double r;	
	double target;//установка
	PID(double r_ = 0, double bias_ = 0.1, double P_ = Pc, double I_ = Ic, double D_ = Dc);
	~PID();
	/// <summary>
	///	Задаёт начальную позицию регулятора, вычисляя integral
	/// </summary>
	/// <param name="pos">  задаваемая начальная позиция</param>
	/// <param name="offset"></param>
	void set_zero_pos(double pos, double offset = 0);
	/// <summary>
	/// Функция вычисления PID сигнала
	/// </summary>
	/// <param name="r">установка</param>
	/// <param name="y">сигнал</param>
	/// <param name="dtime"> время дискретизации </param>
	/// <param name="d_err">производная невязки</param>
	/// <param name="max_step"> максимальный размер шага</param>
	/// <param name="max_err"></param>
	/// <returns> контроллирующий сигнал </returns>
	double signal( double y, double dtime,  double max_step = 0.05, double max_err = 10);
	void reset(double r_, double bias_ = 0.1, double P_ = Pc, double I_ = Ic, double D_ = Dc);
	/// <summary>
	/// 
	/// </summary>
	/// <param name="folder"></param>
	void reset_from_file(string folder = MAIN_FOLDER + "Settings/");
	void save_settings(string folder = MAIN_FOLDER + "Settings/");

 };

class Regulator	
{
	/// <summary>
	/// ожидание с очисткой буфера
	/// </summary>
	/// <param name="card">плата для очистки</param>
	/// <param name="cnt">количество циклов очистки(~100 us на очистку)</param>
	void wait_clear_buf(Card& card, int cnt = 30);
	/// <summary>
	/// # - Название таски
	///# - % выполнения (0-100) с точкой
	///# - секунд до окончания (с точкой)
	///# - x float
	///# - y float
	///# - z float (0.0 - 10.0B)
	///# - Ток float нА
	///# - Voltage DC
	///# - Voltage AC
	///# - Frequency
	/// </summary>
	/// <param name="percent"></param>
	/// <param name="estimated_time"></param>
	void WriteProgressStatus(string name, double percent, double estimated_time);
	/// <summary>
	/// Единичный шаг пьезиков по выбранной оси
	/// </summary>
	/// <param name="axis">ось</param>
	/// <param name="dir">направление(назад/вперёд)</param>
	/// <param name="step_size"> напряжение шага</param>
	/// <param name="step_speed"> скорость шага</param>
	void Step(int axis, int dir, double step_size = 5, double step_speed = 10);
	/// <summary>
	/// Проверка статуса выполнения программы из командного файла (2 - работа, 1 - пауза, 0 - стоп)
	/// </summary>
	/// <param name="folder"></param>
	/// <returns></returns>
	int GetStatus();
	/// <summary>
	/// 
	/// </summary>
	/// <param name="folder"></param>
	bool CheckStatus(string stop_message);
	//void CheckStatus(string message, double& arg = (void*), double bwa = 0.15, string folder = "../../");
	/// <summary>
	/// Запись статуса выполнения программы из командного файла (2 - работа, 1 - пауза, 0 - стоп)
	/// </summary>
	/// <param name="status"></param>
	/// <param name="folder"></param>
	void WriteExecStatus(int status = 2);
	


	/// <summary>
	/// 
	/// </summary>
	/// <param name="folder"></param>
	void ResetPIDFromFile();
	/// <summary>
	/// 
	/// </summary>
	/// <param name="folder"></param>
	void LoadParamsFromFile(string path, int count , double**arr);
	void SaveParamsToFile(string path, int count, ...);

public:
	LCard XYCard;
	NICard ZCard;
	PiezoPositioners piezo;
	ZurichMFLI MFLI;
	PID pid;
	string folder;
	string session_folder;

	/// <summary>
	/// 
	/// </summary>
	/// <param name="path"></param>
	/// <returns></returns>
	string ReadSessionDirectory(string path = SESSION_FILE_NAME);
	/// <summary>
	/// 
	/// </summary>
	/// <param name="path"></param>
	/// <param name="file"></param>
	void AddFileToSession(string path, string file );

	vector<double> buffer = vector<double>(5000, 0);
	double frequency;		
	double noise_limit_V;	//шум конвертера
	double biasDC;			//напряжение на игле DC
	double biasAC;			//напряжение на игле AC
	double sinc_on;
	double filter_freq;
	/// <summary>
	/// Конструктор объектра трегулятора
	/// </summary>
	/// <param name="i_offset">оффсет тока с конвертера с иглы</param>
	/// <param name="noise_limit_V">порог чуствительности к уму конвертера</param>
	/// <param name="frequency">частота конвертера</param>
	/// <param name="bias"></param>
	Regulator(double i_offset = 0, double noise_limit_V = 0.01, string folder= MAIN_FOLDER);
	~Regulator();


	////////////ПЕРЕМЕЩЕНИЕ////////////
	/// <summary>
	/// Плавное перемещение пьезиков 
	/// </summary>
	/// <param name="x_"> координата назначения по оси X</param>
	/// <param name="y_"> координата назначения по оси Y</param>
	/// <param name="step_size"> размер шага плавной развёртки</param>
	void MoveTo(double x_, double y_, double step_size = DEFAULT_MICROSTEP_SIZE, bool need_logs = false);	
	void MoveTo();
	/// <summary>
	/// Плавный возврат пьезиков в (0,0,0)
	/// </summary>
	/// <param name="step"> размер шага плавной развёртки</param>
	void MHome(double step_size = DEFAULT_MICROSTEP_SIZE, bool need_logs = false);	/// <summary>
	/// Плавный возврат пьезиков в (0,0,0)
	/// </summary>
	/// <param name="step"> размер шага плавной развёртки</param>
	void ZHome(double step_size = DEFAULT_MICROSTEP_SIZE);
	/// <summary>
	/// Скачок пьезиков в (0,0,0)
	/// </summary>
	void JHome();
	/// <summary>
	/// Единичный шаг по оси Z
	/// </summary>
	/// <param name="dir">направление: FORWARD true; BACKWARD false </param>
	/// <param name="step_size">размер шага</param>
	void ZStep(int dir, double step_size = 5, bool makelogs = 1);
	/// <summary>
	/// Перемещение шагами по осям XY
	/// </summary>
	/// <param name="x_steps">Количество шагов по оси X</param>
	/// <param name="y_steps">Количество шагов по оси Y</param>
	/// <param name="step_size">размер шага</param>
	void StepXY(int x_steps, int y_steps, bool need_logs = true, double step_size = 5, double step_speed = 10, double delay = 5000, string folder = "../../scans/");
	void StepXY();

	////////////КОНТРОЛИРУЕМОЕ ПЕРЕМЕЩЕНИЕ////////////
	
	/// <summary>
	/// Отвод иглы
	/// </summary>
	/// <param name="steps">Количество шагов ретракта</param>
	/// <param name="touch_v">Напряжение детектирования касания</param>
	/// <param name="step_incr"> инкремент увеличения шага (в случае касания)</param
	void Retract(int steps, double touch_v= 0.05, double step_incr = 0.4);
	void Retract();
	/// <summary>
	/// Предподъём иглы к обазцу
	/// </summary>
	/// <param name="bias_">напряжение на игле</param>
	/// <param name="bwa">размер отскока назад</param>
	/// <param name="target_V">напряжение детектирования касания</param>
	/// <param name="djump">размер шага плавной развёртки</param>
	/// <returns>высоту касания в В</returns>
	double rise( double bwa = 0.1, double target_V = 0.12, double djump = MIN_STEP_SIZE/5);
	/// <summary>
	/// Процедура лэндинга образца
	/// </summary>
	/// <param name="bias_">напряжение на игле</param>
	/// <param name="range">размер скачка</param>
	/// <param name="target_V">ток детектирования касания, 10*нА</param>
	/// <param name="delay_micro">задержка в мкс</param>
	/// <param name="djump">размер шага плавной развёртки</param>
	/// <returns> высота касания в В</returns>
	double Landing();

	////////////PID РЕГУЛЯТОРЫ////////////

	/// <summary>
	/// Регуляция на основе логарифмического ПИД алгоритма
	/// </summary>
	/// <param name="bias_">напряжение на игле, В</param>
	/// <param name="target_V">значение уставки по току</param>
	/// <param name="duration_us">длительность периода регуляции</param>
	/// <param name="pid_log_offset">логарифмическое смещение входного сигнала</param>
	/// <param name="start_offset"> смещение оси Z в начале скана </param>
	void IntPID(double bias_ = 1, double target_V = 0.25, double duration_us = 0, double pid_log_offset = 0.05, double start_offset = -10 * MIN_STEP_SIZE);
	/// <summary>
	/// Регуляция с помощью продвинутого собственного ПИД алгоритма на основе W-функции Ламберта
	/// </summary>
	/// <param name="bias_">напряжение на игле</param>
	/// <param name="target_V">значение уставки по току</param>
	/// <param name="duration_us">длительность периода регуляции</param>
	/// <param name="start_offset"> смещение оси Z в начале скана </param>
	/// <param name="I_to_nA"> коэффициент конвертации сигнала напряжения в ток </param>
	double IntPID_exp(double bias_ = 1, double target_V = 0.25, double duration_us = 0, double start_pos = 0, int polarity = 1,  double touch_lim = -0.0015, int update_delay = 50000);


	////////////ИЗМЕРЕНИЯ И СКАНЫ////////////

	/// <summary>
	/// Измерение набора ШВАХ(шум-вольт-амперной характеристики)
	/// </summary>
	/// <param name="count"> количество измерений</param>
	/// <param name="target_V"> уставка по току </param>
	/// <param name="bias"> напряжение на игле</param>
	/// <param name="delay"> задержка между считываниями, с (должна быть больше чем период сбора)</param>
	/// <param name="pre_wait"> время подвода и входа в режим перед измерениями, секунд</param>
	/// <param name="folder"></param>
	void VANC_PID();
	

	/// <summary>
	/// Сканирование больших площадей по ёмкостному сигналу 
	/// </summary>
	/// <param name="bias_">напряжение на игле, В</param>
	/// <param name="freq">частота баяса</param>
	/// <param name="crit_V">напряжение детектирования касания</param>
	/// <param name="x_steps">количество точек по оси X</param>
	/// <param name="y_steps">количество точек по оси Y</param>
	/// <param name="X_step_sz">размер шага точек по оси X (в шагах пьезоподвижек)</param>
	/// <param name="Y_step_sz">размер шага точек по оси Y (в шагах пьезоподвижек)</param>
	/// <param name="step_V">напряжение шага пьезоподвижек</param>
	/// <param name="I_to_nA"></param>
	/// <param name="folder"></param>
	void CapStepScan(double biasAC, double freq = 5000, double crit_V = 0.25, int x_steps = 50, int y_steps = 50, int X_step_sz = 4, int Y_step_sz = 4, double step_V = 5.0);
	void CapStepScan();
	
	/// <summary>
	/// Сканирование касанием с подъёмом
	/// </summary>	
	/// <param name="bwa"> размах отскока назад при касании</param>
	/// <param name="crit_V">напряжение детектирования касания</param>
	/// <param name="x_start"></param>
	/// <param name="x_step">шаг по оси X</param>
	/// <param name="x_stop">размер скана по оси X</param>
	/// <param name="y_start"></param>
	/// <param name="y_step">шаг по оси Y</param>
	/// <param name="y_stop">размер скана по оси Y</param>
	/// <param name="djump">размер шага плавной развёртки</param>
	/// <param name="up_mult"> дистанция остановки от поверхности при первичном подъёме (в единицах bwa) </param>
	/// <param name="pre_wait"> время ожидания после подъёма перед сканом (для релаксации пьезиков) </param>
	/// <param name="bias_">напряжение на игле, В</param>
	void TouchScan(double biasAC, double bwa = 0.15, double crit_V = 0.25, double x_start = 0, double x_step = 120 * MIN_STEP_SIZE, double x_stop = 15000 * MIN_STEP_SIZE,
		double y_start = 0, double y_step = 120 * MIN_STEP_SIZE, double y_stop = 15000 * MIN_STEP_SIZE, double h_diff_lim = 5, double djump = MIN_STEP_SIZE, double up_mult = 3, double pre_wait = 300);
	void TouchScan();





	////////////ДОП ФУНКЦИИ////////////

	/// <summary>
	/// Калибровка шумогого сигнала с детектора в зависимости от напряжения на гейте калибровочного транзистора.
	/// To calibrate Noise-V(gate) connect Z_coarse(NDAC2) to C1
	/// </summary>
	/// <param name="Vg_min">> минимальный гейт транзистора, В</param>
	/// <param name="Vg_max"> максимальный гейт транзистора, В</param>
	/// <param name="incr"> шаг калибровки гейта транзистора, В</param>
	/// <param name="delay"> задержка миежду измерениями для насыщения конденсаторов, мкс</param>
	/// <param name="dir"> путь сохранения файла</param>
	void Pn_CVg_TransistorCalibration(double Vg_min, double Vg_max = 0.56, double incr = 0.003, int delay = 300000);
	void Pn_CVg_TransistorCalibration();

	/// <summary>
	/// Калибровка сопротивления калибровочного транзистора от напряжения на гейте (по квази-трёхточке).
	/// To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC1) to C2; Noise (F1) -C3
	/// </summary>
	/// <param name="incr"> шаг калибровки гейта транзистора, В</param>
	/// <param name="Vg_min"> минимальный гейт транзистора, В</param>
	/// <param name="Vg_max"> максимальный гейт транзистора, В</param>
	/// <param name="Vsd_crit"> критическое падение напряжения на тразисторе</param>
	/// <param name="Vbias_crit"> максимальное напряжение для подачи на калибровочный вход </param>
	/// <param name="delay_us"> задержка между измерением точек</param>
	/// <param name="dir">путь сохранения файла</param>
	void R_CVg_TransistorCalibration(double incr, double Vg_min = 0.45, double Vg_max = 0.62, double Vsd_crit = 0.1, double  Vbias_crit = 1, int delay_us = 100000);
	void R_CVg_TransistorCalibration();
};

