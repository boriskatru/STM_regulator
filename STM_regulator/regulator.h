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
///#define Pc 0.0001
#define Pc 0.000003
#define Ic 0.0000001
#define Dc 0
#define MIN_STEP_SIZE 0.00015258789 // �������� 0.55 �������� � COARSE � 0.035 �������� � FINE
inline double W_Lambert_approx(double x) {
	double lnxpp = log(x + 1);
	return (0.665 * (1 + 0.0195 * lnxpp) * lnxpp + 0.04);
}
inline double CHTransform(double current, double voltage, double offset = 0.005, double c1 = 2, double c2 = 10) {
	return (W_Lambert_approx(c1 * voltage / max(current, offset)) * c2)* 1.05 * exp(-0.02 / c1 * current);
}
inline double LimCatch(double signal, double limit, double max = 25) {
	if (signal > limit) return signal;
	else return max;
}
inline string get_time_string() {
	time_t rawtime;
	struct tm* timeinfo;
	char buffern[20];								// ������, � ������� ����� ��������� ������� ����
	time(&rawtime);									// ������� ���� � ��������
#pragma warning(suppress : 4996)
	timeinfo = localtime(&rawtime);					// ������� ��������� �����, �������������� � ���������
	strftime(buffern, 20, "%x", timeinfo);
	string time = "";
	time += buffern;
	time += "/";
	strftime(buffern, 20, "%H_%M", timeinfo);

	time += buffern;
	return time;
}
class PID {
	
public:
	double P;
	double I;
	double D;
	double integral;
	double err ; // �������
	double last_signal;
	double tmp;
	PID(double P = Pc, double I = Ic, double D = Dc);
	~PID();
	void set_zero_pos(double pos, double offset = 0);
	/// <summary>
	/// ������� ���������� PID �������
	/// </summary>
	/// <param name="r">���������</param>
	/// <param name="y">������</param>
	/// <param name="dtime"> ����� ������������� </param>
	/// <param name="d_err">����������� �������</param>
	/// <param name="max_step"> ������������ ������ ����</param>
	/// <param name="max_err"></param>
	/// <returns> ��������������� ������ </returns>
	double signal(double r, double y, double dtime, double d_err = 0, double max_step = 0.05, double max_err = 10);
 };

class Regulator	
{
	/// <summary>
	/// �������� � �������� ������
	/// </summary>
	/// <param name="card">����� ��� �������</param>
	/// <param name="cnt">���������� ������ �������(~100 us �� �������)</param>
	void wait_clear_buf(LCard& card, int cnt = 30);
	/// <summary>
	/// ��������� ��� �������� �� ��������� ���
	/// </summary>
	/// <param name="axis">���</param>
	/// <param name="dir">�����������(�����/�����)</param>
	/// <param name="step_size"> ���������� ����</param>
	void Step(int axis, int dir, double step_size = 5);

public:
	LCard  ZCard, XYCard;
	PiezoPositioners piezo;
	PID pid;
	
	vector<double> buffer = vector<double>(5000, 0);
	double frequency;		
	double noise_limit_V;	//��� ����������
	double bias;			//���������� �� ����
	double current_offset;	//������ �� ����

	Regulator(double i_offset = 0, double noise_limit_V = 0.05, double frequency = 10000, double bias = 0.3);
	~Regulator();


	////////////�����������////////////

	/// <summary>
	/// ������� ������� �������� � (0,0,0)
	/// </summary>
	/// <param name="step"> ������ ���� ������� ��������</param>
	void MHome(double step = MIN_STEP_SIZE / 4);
	/// <summary>
	/// ������ �������� � (0,0,0)
	/// </summary>
	void JHome();
	/// <summary>
	/// ��������� ��� �� ��� Z
	/// </summary>
	/// <param name="dir">�����������: FORWARD true; BACKWARD false </param>
	/// <param name="step_size">������ ����</param>
	void ZStep(int dir, double step_size = 5);
	/// <summary>
	/// ����������� ������ �� ���� XY
	/// </summary>
	/// <param name="x_steps">���������� ����� �� ��� X</param>
	/// <param name="y_steps">���������� ����� �� ��� Y</param>
	/// <param name="step_size">������ ����</param>
	void StepXY(int x_steps, int y_steps, double step_size = 5);

	////////////�������������� �����������////////////
	
	/// <summary>
	/// ����� ����
	/// </summary>
	/// <param name="steps">���������� ����� ��������</param>
	/// <param name="step_incr"> ��������� ���������� ���� (� ������ �������)</param>
	/// <param name="rpt">�������� ���� ����� ��� �����������</param>
	void Retract(int steps = 1, double step_incr = 0.4, double rpt = 2);
	/// <summary>
	/// ���������� ���� � ������
	/// </summary>
	/// <param name="bias_">���������� �� ����</param>
	/// <param name="bwa">������ ������� �����</param>
	/// <param name="target_V">���������� �������������� �������</param>
	/// <param name="djump">������ ���� ������� ��������</param>
	/// <returns>������ ������� � �</returns>
	double rise(double bias_ = 3, double bwa = 0.1, double target_V = 0.1, double djump = MIN_STEP_SIZE);
	/// <summary>
	/// ��������� �������� �������
	/// </summary>
	/// <param name="bias_">���������� �� ����</param>
	/// <param name="range">������ ������</param>
	/// <param name="target_V">���������� �������������� �������</param>
	/// <param name="delay_micro">�������� � ���</param>
	/// <param name="djump">������ ���� ������� ��������</param>
	/// <returns> ������ ������� � �</returns>
	double Landing(double bias_ = 1, double range = 4.5, double target_V = 0.05, double delay_micro = 0, double djump = MIN_STEP_SIZE/5);

	////////////PID ����������////////////

	/// <summary>
	/// ��������� �� ������ ���������������� ��� ���������
	/// </summary>
	/// <param name="bias_">���������� �� ����, �</param>
	/// <param name="target_V">�������� ������� �� ����</param>
	/// <param name="duration_us">������������ ������� ���������</param>
	/// <param name="pid_log_offset">��������������� �������� �������� �������</param>
	/// <param name="start_offset"> �������� ��� Z � ������ ����� </param>
	void IntPID(double bias_ = 1, double target_V = 0.25, double duration_us = 0, double pid_log_offset = 0.05, double start_offset = -10 * MIN_STEP_SIZE);
	/// <summary>
	/// ��������� � ������� ������������ ������������ ��� ��������� �� ������ W-������� ��������
	/// </summary>
	/// <param name="bias_">���������� �� ����</param>
	/// <param name="target_V">�������� ������� �� ����</param>
	/// <param name="duration_us">������������ ������� ���������</param>
	/// <param name="start_offset"> �������� ��� Z � ������ ����� </param>
	/// <param name="I_to_nA"> ����������� ����������� ������� ���������� � ��� </param>
	double IntPID_exp(double bias_ = 1, double target_V = 0.25, double duration_us = 0, double start_pos = 0, double I_to_nA = 10, double touch_lim = 0.002);
	/// <summary>
	/// ��������� �� ������ �������� ��� � �������� ����� ������:
	/// </summary>
	/// <param name="bias_"> ���������� �� ���� </param>
	/// <param name="delay">�������� (���) �� ������������ ���</param>
	/// <param name="bwa">������ ������� (�)</param>
	/// <param name="crit_V">���������� ��������� ������� (�)</param>
	/// <param name="slope">�������� �������� (�/�)</param>
	/// <param name="djump"></param>
	void ExtPID(double bias_ = 1, double delay = 25000, double bwa = 0.1, double crit_V = 0.25, double slope = 30, double djump = MIN_STEP_SIZE);

	////////////��������� � �����////////////

	/// <summary>
	/// ��������� ��������� ���(�����-�������� ��������������)
	/// </summary>
	/// <param name="max">max ���������� �� ����</param>
	/// <param name="min">min ���������� �� ����</param>
	/// <param name="step">��� �� ����������</param>
	/// <param name="name">����� ���</param>
	/// <param name="delay_us">�������� � ��� ����� �����������</param>
	/// <returns>���������� ��� � ������� VAC</returns>
	VAC VAC_(double max, double min, double step, int name = 0, double delay_us = 0);
	/// <summary>
	///  ��������� ��������� ����(����-�����-�������� ��������������)
	/// </summary>
	/// <param name="max">max ���������� �� ����</param>
	/// <param name="min">min ���������� �� ����</param>
	/// <param name="step">��� �� ����������</param>
	/// <param name="name">����� ����</param>
	/// <param name="delay_us">��������� ��� ����� �����������</param>
	/// <returns>���������� ���� � ������� VANC</returns>
	VANC VANC_(double max, double min, double step, int name = 0, double delay_us = 0);

	/// <summary>
	///  ������������ �������� � ��������
	/// </summary>
	/// <param name="bias_">���������� �� ����, �</param>
	/// <param name="bwa"> ������ ������� ����� ��� �������</param>
	/// <param name="crit_V">���������� �������������� �������</param>
	/// <param name="x_dim">������ ����� �� ��� X</param>
	/// <param name="y_dim">������ ����� �� ��� Y</param>
	/// <param name="x_step">��� �� ��� X</param>
	/// <param name="y_step">��� �� ��� Y</param>
	/// <param name="djump">������ ���� ������� ��������</param>
	/// <param name="up_mult"> ��������� ��������� �� ����������� ��� ��������� ������� (� �������� bwa) </param>
	void TouchScan(double bias_ = 0.6, double bwa = 0.15, double crit_V = 0.25, double x_dim = 15000 * MIN_STEP_SIZE, double y_dim = 15000 * MIN_STEP_SIZE,
		double x_step = 120 * MIN_STEP_SIZE, double y_step = 120 * MIN_STEP_SIZE, double djump = MIN_STEP_SIZE, int up_mult = 3); 
	/// <summary>
	/// ���� �� ������������� VAC � ������ �����
	/// </summary>
	void VAC_scan();



	////////////��� �������////////////

	/// <summary>
	/// ���������� ���������� (�� �����������)
	/// </summary>
	/// <param name="points_num"></param>
	/// <param name="file"></param>
	/// <param name="filename"></param>
	void Calibration(int points_num, ofstream file, string filename = "Calibration.txt");
	/// <summary>
	/// ������� ���� ���������� �����������. �� ��������� ��� Lock-in method!!!
	/// </summary>
	/// <param name="cnt">���������� ������</param>
	void ClearTip(int cnt = 25);

	/// <summary>
	/// ���������� �������� ������� � ��������� � ����������� �� ���������� �� ����� �������������� �����������
	/// </summary>
	/// <param name="point_num">���������� �����</param>
	/// <param name="offset_V">����������� ���� �����������, �</param>
	/// <param name="incr">��� ���������� ����� �����������, �</param>
	/// <param name="dir">���� ���������� �����</param>
	void R_NV_TransistorCalibration(int point_num = 67, double offset_V = 0.35, double incr = 0.003, string dir="../../scans/");

	/// <summary>
	/// ���������� ������������� �������������� ����������� �� ���������� �� ����� (�� �����-��������)
	/// </summary>
	/// <param name="incr">��� ���������� ����� �����������, �</param>
	/// <param name="Vg_min">����������� ���� �����������, �</param>
	/// <param name="Vg_max">������������ ���� �����������, �</param>
	/// <param name="Vsd_crit">����������� ������� ���������� �� ����������</param>
	/// <param name="delay_us">�������� ����� ���������� �����</param>
	/// <param name="dir">���� ���������� �����</param>
	void R_V_TransistorCalibration(double incr = 0.003, double Vg_min = 0.35, double Vg_max = 0.55, double Vsd_crit = 0.3, int delay_us = 600000, string dir = "../../scans/");
};

