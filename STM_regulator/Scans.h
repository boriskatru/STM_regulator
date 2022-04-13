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
#include "gwyfile.h"
#define FORWARD true
#define BACKWARD false

class VAC {
	/// <summary>
	/// Печать в файл. ПОРЯДОК СТОЛБЦОВ: V,C
	/// (напряжение, ток)
	/// </summary>
	/// <param name="dir"> Напрявление вахи(туда/обратно) </param>
	/// <param name="outstream"> Поток выходного файла </param>
	void print_in_file(bool dir, ofstream& outstream);
	/// <summary>
	/// Усреднение при выводе в командную строку
	/// </summary>
	/// <param name="array">усредняемый массив</param>
	/// <param name="start"> начальная точка</param>
	/// <param name="avrg_num">количество усредняемых точек</param>
	/// <returns>среднее от start до avrg_num) </returns>
	double avrg(vector<double>& array, int start, int avrg_num = 3);
public:
	int points_num;
	vector<double> FWvoltages;
	vector<double> FWcurrents;
	vector<double> BWvoltages;
	vector<double> BWcurrents;
	string save_dir;
	string fwname;
	string bwname;
	VAC(int count);
	/// <summary>
	///  Сохранение конкретной точки на ВАХ
	/// </summary>
	/// <param name="point"> Номер точки </param>
	/// <param name="dir"> Направление текущей развёртки </param>
	/// <param name="data"> Принятые данные </param>
	/// <param name="ch_v"> Номер канала для напряжения</param>
	/// <param name="ch_c"> Номер канала для тока </param>
	void set(int point, bool dir, ADC_Collect data, int ch_c = 0, int ch_v = 1);
	/// <summary>
	/// Вывод в командную строку(для отладки)
	/// </summary>
	/// <param name="period">выборка периода строк</param>
	void print_cout(int period = 3);
	/// <summary>
	/// Печать в файл. ПОРЯДОК СТОЛБЦОВ: V,C
	/// (напряжение, ток)
	/// </summary>
	/// <param name="name"> Имя ВАХи </param>
	void print_(string name);
};

class VANC: public VAC {
	/// <summary>
	/// Печать в файл. ПОРЯДОК СТОЛБЦОВ: V,C,N
	/// (напряжение, ток, мощность шума)
	/// </summary>
	/// <param name="dir"> Напрявление вахи(туда/обратно) </param>
	/// <param name="outstream"> Поток выходного файла </param>
	void print_in_file(bool dir, ofstream& outstream);
public:
	vector<double> FWnoise;
	vector<double> BWnoise;
	VANC(int count);
	/// <summary>
	/// Сохранение конкретной точки на ВАХ
	/// </summary>
	/// <param name="point"> Номер точки </param>
	/// <param name="dir"> Направление текущей развёртки </param>
	/// <param name="data"> Принятые данные </param>
	/// <param name="ch_v"> Номер канала для напряжения</param>
	/// <param name="ch_c"> Номер канала для тока </param>
	/// <param name="ch_n"> Номер канала для шума</param>
	void set(int point, bool dir, ADC_Collect data, int ch_c = 0, int ch_v = 1, int ch_n = 2);
	/// <summary>
	/// Печать в файл. ПОРЯДОК СТОЛБЦОВ: V,C,N
	/// (напряжение, ток, мощность шума)
	/// </summary>
	/// <param name="name"> Имя ВАХи </param>
	void print_(string name);
	
};

class Scan {
public:
	/* размер скана по X (В) */
	double x_dim;
	/* размер скана по Y (В) */
	double y_dim;
	/* шаг скана по X (В) */
	double x_step;
	/* шаг скана по Y (В) */
	double y_step;
	int x_n, y_n;
	vector <vector<double>> HFWplot;
	vector <vector<double>> HBWplot;
	vector <vector<double>> CFWplot;
	vector <vector<double>> CBWplot;
	string save_dir;
	string fwname;
	string bwname;
	/// <summary>
	/// Сканирование методом последовательного тыка
	/// </summary>
	/// <param name="x_dim">Размер скана по X, В</param>
	/// <param name="y_dim">Размер скана по Y, В</param>
	/// <param name="x_step">Размер шага X, В</param>
	/// <param name="y_step">Размер шага Y, В</param>
	Scan(double x_dim, double y_dim, double x_step, double y_step);
	/// <summary>
	/// Сохраниение скана в файл (целиком)
	/// </summary>
	/// <param name="save_d"> директория сохраниения</param>
	void SaveFiles(string save_d = "-");
	/// <summary>
	/// Сохраниение строки скана в файл
	/// </summary>
	/// <param name="row"> номер строки</param>
	/// <param name="save_d">директория сохраниения</param>
	void SaveRow(int row, string save_d = "-");
	
};

class VAC_Scan : public Scan {
	vector<vector<vector<VAC>>> FWVACs;
	vector<vector<vector<VAC>>> BWVACs;
	VAC_Scan(double x_dim, double y_dim, double x_step, double y_step, int VAC_count, int points_count);
	void SaveFiles(string save_d = "-");
};