#include "piezo_positioners.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
using namespace std;
using namespace std::chrono;
/// <summary>
/// ����������/��������� ������� ���������������
/// </summary>
/// <param name="position"> ����� �������</param>
/// <param name="unit"> ������� ��������� (V,U -�����; M,N - ��) </param>
void PiezoPositioners::UpdatePos(Vecter position, char unit ) {

	if ((unit == 'V') || (unit == 'v') || (unit == 'U') || (unit == 'u')) {
		last_move = position - position_V;
		position_V = position;
		position_nm.x_proj = position.x_proj * calibration_constant.x_proj;
		position_nm.y_proj = position.y_proj * calibration_constant.y_proj;
		position_nm.z_proj = position.z_proj * calibration_constant.z_proj;
	}
	else if ((unit == 'M') || (unit == 'm') || (unit == 'N') || (unit == 'n')) {
		last_move = position - position_nm;
		position_nm = position;
		position_V.x_proj = position.x_proj / calibration_constant.x_proj;
		position_V.y_proj = position.y_proj / calibration_constant.y_proj;
		position_V.z_proj = position.z_proj / calibration_constant.z_proj;
	}
	else cout << "Unknown unit. Please use V or M (nanometrs)" << endl;
	if (position_V.x_proj > V_uplimit.x_proj) {
		position_V.x_proj = V_uplimit.x_proj;
		//cout << "X Voltage uplimit exceeded" << endl;
	}
	if (position_V.y_proj > V_uplimit.y_proj) {
		position_V.y_proj = V_uplimit.y_proj;
		//cout << "Y Voltage uplimit exceeded" << endl;
	}
	if (position_V.z_proj > V_uplimit.z_proj) {
		position_V.z_proj = V_uplimit.z_proj;
		//cout << "Z Voltage uplimit exceeded" << endl;
	}
	if (position_V.z_proj < V_downlimit.z_proj) {
		position_V.z_proj = V_downlimit.z_proj;
		//cout << "Z Voltage downlimit exceeded" << endl;
	}
	if (position_V.x_proj < V_downlimit.x_proj) {
		position_V.x_proj = V_downlimit.x_proj;
		//cout << "Z Voltage downlimit exceeded" << endl;
	}
	if (position_V.y_proj < V_downlimit.y_proj) {
		position_V.y_proj = V_downlimit.y_proj;
		//cout << "Z Voltage downlimit exceeded" << endl;
	}
}
/// <summary>
/// ����������/��������� ������� ��������������� �� ��� Z
/// </summary>
/// <param name="positionZ"> ����� ������� �� Z </param>
/// <param name="unit"> ������� ��������� (V,U -�����; M,N - ��) </param>
void PiezoPositioners::UpdatePos(double positionZ, char unit ) {

	if ((unit == 'V') || (unit == 'v') || (unit == 'U') || (unit == 'u')) {
		last_move.z_proj = positionZ - position_V.z_proj;
		position_V.z_proj = positionZ;

		position_nm.z_proj = positionZ * calibration_constant.z_proj;
	}
	else if ((unit == 'M') || (unit == 'm') || (unit == 'N') || (unit == 'n')) {
		last_move.z_proj = positionZ - position_nm.z_proj;
		position_nm.z_proj = positionZ;
		position_V.z_proj = positionZ / calibration_constant.z_proj;
	}
	else cout << "Unknown unit. Please use V or M (nanometrs)" << endl;
	if (position_V.z_proj > V_uplimit.z_proj) {
		position_V.z_proj = V_uplimit.z_proj;
		//cout << "Z Voltage uplimit exceeded" << endl;
	}
	if (position_V.z_proj < V_downlimit.z_proj) {
		position_V.z_proj = V_downlimit.z_proj;
		//cout << "Z Voltage downlimit exceeded" << endl;
	}
}
PiezoPositioners::PiezoPositioners(double _capacity, double _resistivity, double _cutoff_freq, Vecter _calibration) :
	capacity(_capacity),
	resistivity(_resistivity),
	cutoff_freq(_cutoff_freq),
	crit_jump(0),
	calibration_constant(_calibration),
	V_uplimit(5, 5, 5), V_downlimit(0, 0, 0) {}
PiezoPositioners::~PiezoPositioners() {}
/// <summary>
/// ���������� ������� ������� ���������������
/// </summary>
/// <param name="axis"> ��� (X,Y, ��� Z)</param>
/// <param name="unit">������� ��������� (V,U-������; M,N-��)</param>
/// <returns>���������� ������� ������� ���������������</returns>
double PiezoPositioners::Position(char axis , char unit ) {
	if ((unit == 'V') || (unit == 'v') || (unit == 'U') || (unit == 'u')) {
		if ((axis == 'Z') || (axis == 'z'))
			return position_V.z_proj;
		if ((axis == 'X') || (axis == 'x'))
			return position_V.x_proj;
		if ((axis == 'Y') || (axis == 'y'))
			return position_V.y_proj;
	}
	else if ((unit == 'M') || (unit == 'm') || (unit == 'N') || (unit == 'n')) {
		if ((axis == 'Z') || (axis == 'z'))
			return position_nm.z_proj;
		if ((axis == 'X') || (axis == 'x'))
			return position_nm.x_proj;
		if ((axis == 'Y') || (axis == 'y'))
			return position_nm.y_proj;
	}
}
/// <summary>
///  ������ ����������� �� �������� ����������
/// </summary>
/// <param name="step"> ������ �����������</param>
/// <param name="ZCard"> ����� ��� Z </param>
/// <param name="XYCard"> ����� ���� XY</param>
/// <param name="unit"> ������� ���������</param>
void PiezoPositioners::Jump(Vecter step, LCard& ZCard, LCard& XYCard, const char unit ) {
	if ((unit == 'V') || (unit == 'v') || (unit == 'U') || (unit == 'u')) {
		UpdatePos(step + position_V, unit);
	}
	else if ((unit == 'M') || (unit == 'm') || (unit == 'N') || (unit == 'n')) {
		UpdatePos(step + position_nm, unit);
	}
	if (step.z_proj != 0)
		ZCard.SingleAnalogOut(position_V.z_proj, Z_OUT);
	if (step.x_proj != 0)
		XYCard.SingleAnalogOut(position_V.x_proj, X_OUT);
	if (step.y_proj != 0)
		XYCard.SingleAnalogOut(position_V.y_proj, Y_OUT);
}
/// <summary>
/// ������ ����������� � �������� ���������
/// </summary>
/// <param name="position"> ����� ���������� </param>
/// <param name="ZCard"> ����� ��� Z </param>
/// <param name="XYCard"> ����� ���� XY</param>
/// <param name="unit"> ������� ���������</param>
void PiezoPositioners::JumpTo(Vecter position, LCard& ZCard, LCard& XYCard, const char unit) {

	UpdatePos(position, unit);
	if (last_move.z_proj != 0)
		ZCard.SingleAnalogOut(position_V.z_proj, Z_OUT);
	if (last_move.x_proj != 0)
		XYCard.SingleAnalogOut(position_V.x_proj, X_OUT);
	if (last_move.y_proj != 0)
		XYCard.SingleAnalogOut(position_V.y_proj, Y_OUT);
}
/// <summary>
///  ������ ����������� �� �������� ���������� �� ��� Z
/// </summary>
/// <param name="step"> ��������� ����������� �� ��� Z </param>
/// <param name="ZCard"> ����� ��� Z </param>
/// <param name="unit"> ������� ���������</param>
void PiezoPositioners::ZJump(double step, LCard& ZCard, const char unit) {
	if ((unit == 'V') || (unit == 'v') || (unit == 'U') || (unit == 'u')) {
		UpdatePos(step + position_V.z_proj, unit);
	}
	else if ((unit == 'M') || (unit == 'm') || (unit == 'N') || (unit == 'n')) {
		UpdatePos(step + position_nm.z_proj, unit);
	}
	if (step != 0)
		ZCard.SingleAnalogOut(position_V.z_proj, Z_OUT);

}
/// <summary>
/// ������ ����������� � �������� ��������� �� ��� Z
/// </summary>
/// <param name="position"> ����� ���������� ��� Z </param>
/// <param name="ZCard"> ����� ��� Z </param>
/// <param name="unit"> ������� ���������</param>
void PiezoPositioners::ZJumpTo(double position, LCard& ZCard, const char unit) {

	UpdatePos(position, unit);
	if (last_move.z_proj != 0)
		ZCard.SingleAnalogOut(position_V.z_proj, Z_OUT);

}
/// <summary>
///  ������� ����������� �� �������� ����������
/// </summary>
/// <param name="distance">������ �����������</param>
/// <param name="delay_micro"> �������� ����� �����������, ���</param>
/// <param name="djump"> ����������� ������ ���� �� ���</param>
/// <param name="ZCard"> ����� ��� Z </param>
/// <param name="XYCard"> ����� ���� XY</param>
/// <param name="check">�������/������� �������� ����������� �����������</param>
void PiezoPositioners::Move(Vecter distance, double delay_micro, double djump, LCard& ZCard, LCard& XYCard, double (*check)(double) ) {
	Vecter final = (position_V + distance);
	int steps = trunc((distance / djump).len());
	for (int i = 0; i < steps; i++) {
		JumpTo(position_V + distance.Normalize() * djump, ZCard, XYCard);
		uwait(delay_micro);
	}
	JumpTo(final, ZCard, XYCard);
}
/// <summary>
///  ������� ����������� � �������� ���������
/// </summary>
/// <param name="distance">������ �����������</param>
/// <param name="delay_micro"> �������� ����� �����������, ���</param>
/// <param name="djump"> ����������� ������ ���� �� ���</param>
/// <param name="ZCard"> ����� ��� Z </param>
/// <param name="XYCard"> ����� ���� XY</param>
/// <param name="check">�������/������� �������� ����������� �����������</param>
void PiezoPositioners::MoveTo(Vecter destination, double delay_micro, double djump, LCard& ZCard, LCard& XYCard, double (*check)(double) ) {
	Vecter relative = destination - position_V;
	int steps = trunc((relative / djump).len());
	for (int i = 0; i < steps; i++) {
		JumpTo(position_V + relative.Normalize() * djump, ZCard, XYCard);
		uwait(delay_micro);
		//cout << "X: " << position_V.x_proj << "Y: " << position_V.x_proj << endl;
	}
	JumpTo(destination, ZCard, XYCard);
}
