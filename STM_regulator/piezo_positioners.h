#pragma once
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "wait_bh.h"

#include "vecters.h"
#include "LCard.h"

#define Z_COARSE_PREC 0.05
#define Z_OUT L502_DAC_CH1
//#define Z_OUT_FINE L502_DAC_CH2 // Или FINE регулировка, или ВАХ
#define BIAS_OUT L502_DAC_CH2
#define X_OUT L502_DAC_CH1
#define Y_OUT L502_DAC_CH2
#define X_ L502_DAC_CH1
#define Y_ L502_DAC_CH2

using namespace std;
using namespace std::chrono;
/// <summary>
/// Пьезопозиционеры
/// </summary>
class PiezoPositioners
{
	Vecter last_move;
	Vecter position_V;
	Vecter position_nm;
	Vecter calibration_constant; // nm/V
	
	void UpdatePos(Vecter position, char unit = 'V');
	
	void UpdatePos(double positionZ, char unit = 'V');
public:
	double capacity;
	double resistivity;
	double cutoff_freq;
	double crit_jump;
	Vecter V_uplimit, V_downlimit;
	PiezoPositioners(double _capacity = 0.000001, double _resistivity = 0, double _cutoff_freq = 200000, Vecter _calibration = Vecter(300, 750, 750));
	~PiezoPositioners();
	
	double Position(char axis = 'Z', char unit = 'V');
	
	void Jump(Vecter step, LCard& ZCard, LCard& XYCard, const char unit = 'V');
	
	void JumpTo(Vecter position, LCard& ZCard, LCard& XYCard, const char unit = 'V');
	void ZJump(double step, LCard& ZCard, const char unit = 'V');
	void ZJumpTo(double position, LCard& ZCard, const char unit = 'V');
	/*void ZFJumpTo(double position, LCard& ZCard,  const char unit = 'V') {

		UpdatePos(position, unit);
		tmp = fmod(position_V.z_proj, Z_COARSE_PREC);
		if (last_move.z_proj != 0)
			ZCard.SingleAnalogOut(position-tmp, Z_OUT);
			ZCard.SingleAnalogOut(15*tmp, Z_OUT_FINE);

	}*/
	void Move(Vecter distance, double delay_micro, double djump, LCard& ZCard, LCard& XYCard, double (*check)(double) = NULL);
	void MoveTo(Vecter destination, double delay_micro, double djump, LCard& ZCard, LCard& XYCard, double (*check)(double) = NULL);
	
};

