#pragma once
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <l502api.h>
#include <filesystem>


#include "piezo_positioners.h"
#include "LCard.h"
#include "vecters.h"
#include "wait_bh.h"
#include "Scans.h"
#include "gwyfile.h"

using namespace std;

#define FORWARD true
#define BACKWARD false
#define Pc 0.0001
#define Ic 0.00000001
#define Dc 0
#define MIN_STEP_SIZE 0.00015258789 // примерно 0.5722 ангстрем
inline double W_Lambert_approx(double x) {
	double lnxpp = log(x + 1);
	return (0.665 * (1 + 0.0195 * lnxpp) * lnxpp + 0.04);
}
inline double CHTransform(double current, double voltage, double offset = 0.005, double c1 = 2, double c2 = 10) {
	return (W_Lambert_approx(c1 * voltage / max(current, offset)) * c2)* 1.05 * exp(-0.02 / c1 * current);
}

class PID {
	
public:
	double P;
	double I;
	double D;
	double integral;
	double err ; // невязка
	double last_signal;
	double tmp;
	PID(double P = Pc, double I = Ic, double D = Dc);
	~PID();
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
	double signal(double r, double y, double dtime, double d_err = 0, double max_step = 0.05, double max_err = 10);
 };

class Regulator	
{
	/// <summary>
	/// ожидание с очисткой буфера
	/// </summary>
	/// <param name="card">плата для очистки</param>
	/// <param name="cnt">количество циклов очистки(~100 us на очистку)</param>
	void wait_clear_buf(LCard& card, int cnt = 30);
	/// <summary>
	/// Единичный шаг пьезиков по выбранной оси
	/// </summary>
	/// <param name="axis">ось</param>
	/// <param name="dir">направление(назад/вперёд)</param>
	/// <param name="step_size"> напряжение шага</param>
	void Step(int axis, int dir, double step_size = 5);

public:
	LCard  ZCard, XYCard;
	PiezoPositioners piezo;
	PID pid;
	
	vector<double> buffer = vector<double>(5000, 0);
	double frequency;		
	double noise_limit_V;	//шум конвертера
	double bias;			//напряжение на игле
	double current_offset;	//оффсет по току

	Regulator(double i_offset = 0, double noise_limit_V = 0.05, double frequency = 10000, double bias = 0.3);
	~Regulator();



	////////////ПЕРЕМЕЩЕНИЕ////////////

	/// <summary>
	/// Плавный возврат пьезиков в (0,0,0)
	/// </summary>
	/// <param name="step"> размер шага плавной развёртки</param>
	void MHome(double step = MIN_STEP_SIZE / 4);
	/// <summary>
	/// Скачок пьезиков в (0,0,0)
	/// </summary>
	void JHome();
	/// <summary>
	/// Единичный шаг по оси Z
	/// </summary>
	/// <param name="dir">направление: FORWARD true; BACKWARD false </param>
	/// <param name="step_size">размер шага</param>
	void ZStep(int dir, double step_size = 5);
	/// <summary>
	/// Перемещение шагами по осям XY
	/// </summary>
	/// <param name="x_steps">Количество шагов по оси X</param>
	/// <param name="y_steps">Количество шагов по оси Y</param>
	/// <param name="step_size">размер шага</param>
	void StepXY(int x_steps, int y_steps, double step_size = 5);

	////////////КОНТРОЛИРУЕМОЕ ПЕРЕМЕЩЕНИЕ////////////
	
	/// <summary>
	/// Отвод иглы
	/// </summary>
	/// <param name="steps">Количество шагов ретракта</param>
	/// <param name="step_incr"> инкремент увеличения шага (в случае касания)</param>
	/// <param name="rpt">повторов шага между его увеличением</param>
	void Retract(int steps = 1, double step_incr = 0.5, double rpt = 2);
	/// <summary>
	/// Предподъём иглы к обазцу
	/// </summary>
	/// <param name="bias_">напряжение на игле</param>
	/// <param name="bwa">размер отскока назад</param>
	/// <param name="target_V">напряжение детектирования касания</param>
	/// <param name="djump">размер шага плавной развёртки</param>
	/// <returns>высоту касания в В</returns>
	double rise(double bias_ = 3, double bwa = 0.1, double target_V = -0.15, double djump = MIN_STEP_SIZE);
	/// <summary>
	/// Процедура лэндинга образца
	/// </summary>
	/// <param name="bias_">напряжение на игле</param>
	/// <param name="range">размер скачка</param>
	/// <param name="target_V">напряжение детектирования касания</param>
	/// <param name="delay_micro">задержка в мкс</param>
	/// <param name="djump">размер шага плавной развёртки</param>
	/// <returns> высота касания в В</returns>
	double Landing(double bias_ = 1, double range = 4.5, double target_V = -0.25, double delay_micro = 0, double djump = MIN_STEP_SIZE);

	////////////PID РЕГУЛЯТОРЫ////////////

	/// <summary>
	/// Регуляция на основе логарифмического ПИД алгоритма
	/// </summary>
	/// <param name="bias_">напряжение на игле, В</param>
	/// <param name="target_V">значение уставки по току</param>
	/// <param name="duration_us">длительность периода регуляции</param>
	/// <param name="pid_log_offset">логарифмическое смещение входного сигнала</param>
	/// <param name="start_offset"> смещение оси Z в начале скана </param>
	void IntPID(double bias_ = 1, double target_V = -0.2, double duration_us = 0, double pid_log_offset = 0.05, double start_offset = -10 * MIN_STEP_SIZE);
	/// <summary>
	/// Регуляция с помощью продвинутого собственного ПИД алгоритма на основе W-функции Ламберта
	/// </summary>
	/// <param name="bias_">напряжение на игле</param>
	/// <param name="target_V">значение уставки по току</param>
	/// <param name="duration_us">длительность периода регуляции</param>
	/// <param name="start_offset"> смещение оси Z в начале скана </param>
	/// <param name="I_to_nA"> коэффициент конвертации сигнала напряжения в ток </param>
	void IntPID_exp(double bias_ = 1, double target_V = -0.2, double duration_us = 0, double start_offset = -10 * MIN_STEP_SIZE, double I_to_nA = 10);
	/// <summary>
	/// регуляция на основе внешнего ПИД с подъёмом между шагами:
	/// </summary>
	/// <param name="bias_"> напрядение на игле </param>
	/// <param name="delay">задержка (мкс) на стабилизацию ПИД</param>
	/// <param name="bwa">высота подъёма (В)</param>
	/// <param name="crit_V">напряжение остановки подъёма (В)</param>
	/// <param name="slope">скорость поднятия (В/с)</param>
	/// <param name="djump"></param>
	void ExtPID(double bias_ = 1, double delay = 25000, double bwa = 0.1, double crit_V = -0.15, double slope = 30, double djump = MIN_STEP_SIZE);

	////////////ИЗМЕРЕНИЯ И СКАНЫ////////////

	/// <summary>
	/// Измерение единичной ВАХ(вольт-амперной характеристики)
	/// </summary>
	/// <param name="max">max напряжение на игле</param>
	/// <param name="min">min напряжение на игле</param>
	/// <param name="step">шаг по напряжению</param>
	/// <param name="name">номер ВАХ</param>
	/// <param name="delay_us">задержка в мкс между измерениями</param>
	/// <returns>Возвращает ВАХ в формате VAC</returns>
	VAC VAC_(double max, double min, double step, int name = 0, double delay_us = 0);
	/// <summary>
	///  Измерение единичной ШВАХ(шумо-вольт-амперной характеристики)
	/// </summary>
	/// <param name="max">max напряжение на игле</param>
	/// <param name="min">min напряжение на игле</param>
	/// <param name="step">шаг по напряжению</param>
	/// <param name="name">номер ШВАХ</param>
	/// <param name="delay_us">задержкав мкс между измерениями</param>
	/// <returns>Возвращает ШВАХ в формате VANC</returns>
	VANC VANC_(double max, double min, double step, int name = 0, double delay_us = 0);

	/// <summary>
	///  Сканирование касанием с подъёмом
	/// </summary>
	/// <param name="bias_">напряжение на игле, В</param>
	/// <param name="bwa"> размер отскока назад при касании</param>
	/// <param name="crit_V">напряжение детектирования касания</param>
	/// <param name="x_dim">размер скана по оси X</param>
	/// <param name="y_dim">размер скана по оси Y</param>
	/// <param name="x_step">шаг по оси X</param>
	/// <param name="y_step">шаг по оси Y</param>
	/// <param name="djump">размер шага плавной развёртки</param>
	/// <param name="up_mult"> дистанция остановки от поверхности при первичном подъёме (в единицах bwa) </param>
	void TouchScan(double bias_ = 0.6, double bwa = 0.15, double crit_V = -0.015, double x_dim = 15000 * MIN_STEP_SIZE, double y_dim = 15000 * MIN_STEP_SIZE,
		double x_step = 120 * MIN_STEP_SIZE, double y_step = 120 * MIN_STEP_SIZE, double djump = MIN_STEP_SIZE, int up_mult = 3); 
	/// <summary>
	/// Скан со сканированием VAC в каждой точке
	/// </summary>
	void VAC_scan();



	////////////ДОП ФУНКЦИИ////////////

	/// <summary>
	/// Калибровка регулятора (НЕ РЕАЛИЗОВАНО)
	/// </summary>
	/// <param name="points_num"></param>
	/// <param name="file"></param>
	/// <param name="filename"></param>
	void Calibration(int points_num, ofstream file, string filename = "Calibration.txt");
	/// <summary>
	/// Очистка иглы переменным напряжением
	/// </summary>
	/// <param name="cnt">количество циклов</param>
	void ClearTip(int cnt = 25);
};

