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
	/// ������ � ����. ������� ��������: V,C
	/// (����������, ���)
	/// </summary>
	/// <param name="dir"> ����������� ����(����/�������) </param>
	/// <param name="outstream"> ����� ��������� ����� </param>
	void print_in_file(bool dir, ofstream& outstream);
	/// <summary>
	/// ���������� ��� ������ � ��������� ������
	/// </summary>
	/// <param name="array">����������� ������</param>
	/// <param name="start"> ��������� �����</param>
	/// <param name="avrg_num">���������� ����������� �����</param>
	/// <returns>������� �� start �� avrg_num) </returns>
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
	///  ���������� ���������� ����� �� ���
	/// </summary>
	/// <param name="point"> ����� ����� </param>
	/// <param name="dir"> ����������� ������� �������� </param>
	/// <param name="data"> �������� ������ </param>
	/// <param name="ch_v"> ����� ������ ��� ����������</param>
	/// <param name="ch_c"> ����� ������ ��� ���� </param>
	void set(int point, bool dir, ADC_Collect data, int ch_c = 0, int ch_v = 1);
	/// <summary>
	/// ����� � ��������� ������(��� �������)
	/// </summary>
	/// <param name="period">������� ������� �����</param>
	void print_cout(int period = 3);
	/// <summary>
	/// ������ � ����. ������� ��������: V,C
	/// (����������, ���)
	/// </summary>
	/// <param name="name"> ��� ���� </param>
	void print_(string name);
};

class VANC: public VAC {
	/// <summary>
	/// ������ � ����. ������� ��������: V,C,N
	/// (����������, ���, �������� ����)
	/// </summary>
	/// <param name="dir"> ����������� ����(����/�������) </param>
	/// <param name="outstream"> ����� ��������� ����� </param>
	void print_in_file(bool dir, ofstream& outstream);
public:
	vector<double> FWnoise;
	vector<double> BWnoise;
	VANC(int count);
	/// <summary>
	/// ���������� ���������� ����� �� ���
	/// </summary>
	/// <param name="point"> ����� ����� </param>
	/// <param name="dir"> ����������� ������� �������� </param>
	/// <param name="data"> �������� ������ </param>
	/// <param name="ch_v"> ����� ������ ��� ����������</param>
	/// <param name="ch_c"> ����� ������ ��� ���� </param>
	/// <param name="ch_n"> ����� ������ ��� ����</param>
	void set(int point, bool dir, ADC_Collect data, int ch_c = 0, int ch_v = 1, int ch_n = 2);
	/// <summary>
	/// ������ � ����. ������� ��������: V,C,N
	/// (����������, ���, �������� ����)
	/// </summary>
	/// <param name="name"> ��� ���� </param>
	void print_(string name);
	
};

class Scan {
public:
	/* ������ ����� �� X (�) */
	double x_dim;
	/* ������ ����� �� Y (�) */
	double y_dim;
	/* ��� ����� �� X (�) */
	double x_step;
	/* ��� ����� �� Y (�) */
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
	/// ������������ ������� ����������������� ����
	/// </summary>
	/// <param name="x_dim">������ ����� �� X, �</param>
	/// <param name="y_dim">������ ����� �� Y, �</param>
	/// <param name="x_step">������ ���� X, �</param>
	/// <param name="y_step">������ ���� Y, �</param>
	Scan(double x_dim, double y_dim, double x_step, double y_step);
	/// <summary>
	/// ����������� ����� � ���� (�������)
	/// </summary>
	/// <param name="save_d"> ���������� �����������</param>
	void SaveFiles(string save_d = "-");
	/// <summary>
	/// ����������� ������ ����� � ����
	/// </summary>
	/// <param name="row"> ����� ������</param>
	/// <param name="save_d">���������� �����������</param>
	void SaveRow(int row, string save_d = "-");
	
};

class VAC_Scan : public Scan {
	vector<vector<vector<VAC>>> FWVACs;
	vector<vector<vector<VAC>>> BWVACs;
	VAC_Scan(double x_dim, double y_dim, double x_step, double y_step, int VAC_count, int points_count);
	void SaveFiles(string save_d = "-");
};