#include "regulator.h"

PID::PID(double P, double I, double D ) :
	P(P),
	I(I),
	D(D), err(0),
	integral(0),last_signal(0),tmp(0) {

}
PID::~PID() {
	P = I = D = 0;
}
void PID::set_zero_pos(double pos, double offset ) {
	integral = (pos + offset) / I;
}
double PID::signal(double r, double y, double dtime, double d_err, double max_step, double max_err ) {
	err = y - r;
	if (abs(err) > max_err) err = sign(err * max_err);
	integral += err * dtime;
	if (integral <= 0)  integral = 0;

	tmp = P * err + I * integral + D * d_err;
	/*if ((tmp - last_signal) > max_step) {
		last_signal += max_step;
	}
	else if ((tmp - last_signal) < max_step) {
		last_signal -= max_step;
	}
	else*/ last_signal = tmp;
	return last_signal;
}


void Regulator::wait_clear_buf(LCard& card, int cnt ) {
	for (int i = 0; i < cnt; i++)  card.AnalogRead();
}

void Regulator::Step(int axis, int dir, double step_size ) {
	Vecter step;
	if (!axis)
		step = Vecter(step_size, 0, 0);
	else
		step = Vecter(0, step_size, 0);
	cout << axis << endl;
	if (dir > 0) {
		piezo.MoveTo(step, 0, MIN_STEP_SIZE, ZCard, XYCard);
		JHome();
	}
	else if (dir < 0) {
		piezo.JumpTo(step, ZCard, XYCard);
		MHome(MIN_STEP_SIZE);
	}
}


Regulator::Regulator(double i_offset, double noise_limit_V, double frequency , double bias) :
	XYCard(1, 4, ADC_BUF_SIZE_2),
	ZCard(2, 1),
	noise_limit_V(noise_limit_V),
	frequency(frequency),
	current_offset(i_offset), bias(bias) {
	ZCard.SingleAnalogOut(bias, BIAS_OUT);
	//ZCard.AnalogRead();
	//cout <<"Current signul: "<< ZCard.data.average<<endl;

}
Regulator::~Regulator() {
	MHome();
	ZCard.SingleAnalogOut(0, 0);
	//ZCard.SingleAnalogOut(0, 1);
	ZCard.~LCard(); XYCard.~LCard();
	piezo.~PiezoPositioners();
}


void Regulator::MHome(double step) {
	piezo.MoveTo(Vecter(0, 0, 0), 0, step, ZCard, XYCard);
}
void Regulator::JHome() {
	piezo.JumpTo(Vecter(0, 0, 0), ZCard, XYCard);
}
void Regulator::ClearTip(int cnt) {
	for (int i = 0; i < cnt; i++) {
		VAC_(+5, -5, 0.005);
	}
	ZCard.StopReadStream();
}

void Regulator::ZStep(int dir, double step_size) {
	if (dir > 0) {
		for (double i = 0; i < step_size; i += MIN_STEP_SIZE) {
			ZCard.SingleAnalogOut(i);
		}
		ZCard.SingleAnalogOut(0);
	}
	else {
		for (double i = step_size; i > 0; i -= MIN_STEP_SIZE) {
			ZCard.SingleAnalogOut(i);
		}

	}

}
void Regulator::StepXY(int x_steps, int y_steps, double step_size) {
	for (int i = 0; i < abs(x_steps); i++) Step(X_, x_steps, step_size);
	for (int i = 0; i < abs(y_steps); i++) Step(Y_, y_steps, step_size);
}



void Regulator::Retract(int steps, double step_incr, double rpt) {
	int step = -3;
	MHome();
	double st_sz = max(rise() / 1.25 - 0.2, 0.4);
	MHome();
	wait_clear_buf(ZCard);
	for (double sz = st_sz; sz <= 5; sz += step_incr) {
		for (int i = 0; i < rpt; i++) {
			ZCard.SingleAnalogOut(5);
			ZStep(-1, sz);
			step += 1;
			//uwait(100000);
		}
		//cout << "backstep " << sz << endl;

	}
	while (step < steps) {
		ZStep(-1);
		step++;
	}
}
double Regulator::rise(double bias_, double bwa, double target_V, double djump) {
	bias = bias_;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	//uwait(1000000);
	/*Обнуляем всё*/
	bool is_touch = false;
	int last_height;
	//MHome();
	/*Касание и выдержка*/
	uwait(500000);
	ZCard.StopReadStream();
	for (int i = 0; i < 60; i++) {//считывает n раз для очистки буфера
		ZCard.AnalogRead();
		is_touch = (ZCard.data.Average() > target_V);

	}

	while (!is_touch) {

		piezo.ZJump(djump, ZCard);
		last_height = piezo.Position('Z');
		//stp_count++;
		//uwait(delay_micro);

		is_touch = (ZCard.AnalogRead().Average() > target_V);

		if (last_height < 0.05) is_touch = false;
		if (piezo.Position('Z') >= 5) {
			//uwait(delay_micro);
			MHome();
			break;
		}
	}

	ZCard.StopReadStream();
	piezo.Move(Vecter(0, 0, -bwa), 0, 5 * MIN_STEP_SIZE, ZCard, XYCard);
	MHome();
	//uwait(1000000);
	return last_height;
}
double Regulator::Landing(double bias_, double range, double target_V, double delay_micro, double djump) {
	bias = bias_;
	target_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	bool is_touch = false;
	int stp_count = 0;
	double last_height = 99;
	Vecter zero(0, 0, 0);
	for (int i = 0; i < 60; i++) {//считывает n раз для очистки буфера
		ZCard.AnalogRead();
		is_touch = (ZCard.data.Average() > target_V);
		//cout << ZCard.data.Average(8, 0) << endl;
	}
	//getchar(); getchar();
	while (!is_touch) {

		piezo.ZJump(djump, ZCard);
		last_height = piezo.Position('Z');
		//stp_count++;
		//uwait(delay_micro);

		is_touch = (ZCard.AnalogRead().Average() > target_V);

		if (last_height < 0.2) is_touch = false;
		if (piezo.Position('Z') >= range) {
			//uwait(delay_micro);
			piezo.ZJumpTo(0, ZCard);
			for (int i = 0; i < 50; i++) {
				uwait(500);
				ZCard.AnalogRead();
			}


		}
	}

	ZCard.StopReadStream();
	piezo.Move(Vecter(0, 0, -0.1), delay_micro, djump, ZCard, XYCard);
	MHome();
	piezo.JumpTo(Vecter(0, 0, 0), ZCard, XYCard);
	cout << "Touch current signal: " << ZCard.data.Average(8, 0) << endl;
	cout << "Touch_height: " << last_height << endl;
	cout << "Landing done" << endl;

	getchar(); getchar();
	ZCard.StopReadStream();
	return last_height;
}
void Regulator::Calibration(int points_num, ofstream file, string filename) {
	///not realized
}




void Regulator::IntPID(double bias_, double target_V, double duration_us, double pid_log_offset, double start_offset) {
	//int i = 0;
	//piezo.Move(Vecter(0, 0, Landing()-0.01), 0, djump, ZCard, XYCard);
	//piezo.Move(Vecter(0, 0, 0), 0, djump, ZCard, XYCard);
	bias = bias_;
	target_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	ZCard.AnalogRead();

	Timer tmr;
	if (duration_us == 0) {
		while (true) {
			ZCard.AnalogRead();
			piezo.ZJumpTo(pid.signal(log(pid_log_offset - target_V), log(pid_log_offset - ZCard.data.Average()), tmr.get_loop_interval()), ZCard);
		}
	}
	else {
		pid.set_zero_pos(piezo.Position(), start_offset);
		while (tmr.get_full_interval() <= duration_us) {
			ZCard.AnalogRead();
			piezo.ZJumpTo(pid.signal(log(pid_log_offset - target_V), log(pid_log_offset - ZCard.data.Average()), tmr.get_loop_interval()), ZCard);
		}
	}



}
double Regulator::IntPID_exp(double bias_, double target_V, double duration_us, double start_pos, double I_to_nA, double touch_lim) {

	bias = bias_;
	target_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	/*double I_ = ZCard.data.Average() * 10;
	double NL_ = 0.005;
	double I_max = 100;
	double x_c = 10 * W_Lambert_approx(2 / (target_V * 10));
	double x_min = 10 * W_Lambert_approx(2 / NL_);
	double x_max = 10 * W_Lambert_approx(2 / I_max);
	double k_ = (x_c - x_min) / (x_max - x_c);
	double offset = 0;*/
	Timer tmr;
	if (duration_us == 0) {
		pid.set_zero_pos(start_pos);
		while (true) {
			piezo.ZFJumpTo(pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(ZCard.AnalogRead().Average(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval()), ZCard);
		}
	}
	else {
		pid.set_zero_pos(start_pos);
		tmr.get_loop_interval();
		while (tmr.get_full_interval() <= duration_us) {
			piezo.ZFJumpTo(pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(ZCard.AnalogRead().Average(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval()), ZCard);
		}
		
	}

	return pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(ZCard.AnalogRead().Average(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval());

}
void Regulator::ExtPID(double bias_, double delay, double bwa, double crit_V, double slope, double djump) {}



/////////

VAC Regulator::VAC_(double max, double min, double step, int name, double delay_us) {
	VAC  vac((max - min) / step);
	//ZCard.SingleAnalogOut(min, BIAS_OUT);
	for (int i = 0; i < 50; i++)  XYCard.AnalogRead();
	for (int i = (max - min) / step; i > 0; i--) {
		ZCard.SingleAnalogOut(min + i * step, BIAS_OUT);
		if (delay_us != 0)uwait(delay_us);
		vac.set_point(i, BACKWARD, XYCard.AnalogRead());

	}
	for (int i = 0; i < 20; i++)  XYCard.AnalogRead();
	for (int i = 0; i < (max - min) / step; i++) {
		ZCard.SingleAnalogOut(min + i * step, BIAS_OUT);
		if (delay_us != 0)uwait(delay_us);
		vac.set_point(i, FORWARD, XYCard.AnalogRead());;

	}

	ZCard.SingleAnalogOut(bias, BIAS_OUT);
	vac.print_(to_string(name));
	return vac;
}
VANC Regulator::VANC_(double max, double min, double step, int name, double delay_us) {
	VANC  vanc((max - min) / step);

	//ZCard.SingleAnalogOut(min, BIAS_OUT);
	ADC_Collect data;
	for (int i = 0; i < 40; i++)  XYCard.AnalogRead();

	for (int i = (max - min) / step; i > 0; i--) {
		ZCard.SingleAnalogOut(min + i * step, BIAS_OUT);
		if (delay_us != 0)uwait(delay_us);
		vanc.set_point(i, BACKWARD, XYCard.AnalogRead());
	}

	for (int i = 0; i < 20; i++)  XYCard.AnalogRead();

	for (int i = 0; i < (max - min) / step; i++) {
		ZCard.SingleAnalogOut(min + i * step, BIAS_OUT);
		if (delay_us != 0)uwait(delay_us);
		vanc.set_point(i, FORWARD, XYCard.AnalogRead());
	}
	ZCard.SingleAnalogOut(bias, BIAS_OUT);

	vanc.print_(to_string(name));

	return vanc;
}
void Regulator::TouchScan(double bias_ , double bwa, double crit_V, double x_dim, double y_dim,	double x_step, double y_step, double djump, int up_mult) {
	bias = bias_;
	crit_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	uwait(1000000);
	Scan scan(x_dim, y_dim, x_step, y_step);
	/*Обнуляем всё*/
	bool is_touch = false;
	piezo.MoveTo(Vecter(0, 0, 0), 0, djump, ZCard, XYCard);
	/*Касание и выдержка*/
	uwait(1000000);
	ZCard.StopReadStream();
	for (int i = 0; i < 1000; i++)  ZCard.AnalogRead(20);
	cout << "input	" << ZCard.AnalogRead().Average(20) << endl;
	while (!is_touch) {
		piezo.ZJump(djump / 2, ZCard);
		is_touch = (ZCard.AnalogRead().Average(8) < crit_V);

		if (piezo.Position() < 2 * bwa) is_touch = 0;
		for (int i = 0; i < 10; i++)  ZCard.AnalogRead();
		ZCard.AnalogRead();
	}
	while (is_touch) {

		piezo.ZJump(-2 * djump, ZCard);
		is_touch = (ZCard.AnalogRead().Average(20) < crit_V);

		if (piezo.Position() < 2 * bwa) is_touch = 0;
		ZCard.AnalogRead(20);
	}
	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, djump, ZCard, XYCard);
	uwait(60000000); // ждём 60 сек

	ZCard.AnalogRead();
	is_touch = (ZCard.AnalogRead().Average(16) < crit_V);
	while (is_touch) {

		piezo.ZJump(-2 * djump, ZCard);
		is_touch = (ZCard.AnalogRead().Average(16) < crit_V);

		if (piezo.Position() < bwa) is_touch = 0;
		ZCard.AnalogRead(16);
	}
	//piezo.Move(Vecter(0, 0, -bwa), 0, djump, ZCard, XYCard);
	is_touch = (ZCard.AnalogRead().Average(16) < crit_V);

	/*Начало сканирования*/
	for (int y = 0; y < scan.y_n; y++) {
		/*передний ход:*/
		for (int x = 0; x < scan.x_n; x++) {

			for (int i = 0; i < 50; i++) {
				//uwait(50);
				is_touch = (ZCard.AnalogRead().Average() < crit_V);

			}
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {
				piezo.ZJump(djump, ZCard);
				is_touch = (ZCard.AnalogRead().Average() < crit_V);

				if (piezo.Position() < bwa) is_touch = 0;

			}
			scan.HFWplot[y][x] = piezo.Position();
			scan.CFWplot[y][x] = ZCard.data.Average() * 1000;

			//uwait(100);

			while (is_touch) {

				piezo.ZJump(-2 * djump, ZCard);
				is_touch = (ZCard.AnalogRead().Average() < crit_V);

				if (piezo.Position() < bwa) is_touch = 0;
				ZCard.AnalogRead();
			}
			piezo.Move(Vecter(x_step, 0, -bwa), 0, 2 * djump, ZCard, XYCard);
		}
		/*задний ход:*/
		for (int x = scan.x_n - 1; x >= 0; x--) {
			for (int i = 0; i < 50; i++) {
				//uwait(50);
				is_touch = (ZCard.AnalogRead().Average() < crit_V);

			}
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {

				piezo.ZJump(djump, ZCard);
				is_touch = (ZCard.AnalogRead().Average() < crit_V);

				if (piezo.Position() < bwa) is_touch = 0;

			}

			scan.HBWplot[y][x] = piezo.Position();
			scan.CBWplot[y][x] = ZCard.data.Average() * 1000;

			//uwait(100);

			while (is_touch) {

				piezo.ZJump(-2 * djump, ZCard);
				is_touch = (ZCard.AnalogRead().Average() < crit_V);

				if (piezo.Position() < bwa) is_touch = 0;
				ZCard.AnalogRead();
			}
			piezo.Move(Vecter(-x_step, 0, -bwa), 0, 2 * djump, ZCard, XYCard);
		}

		piezo.Move(Vecter(0, y_step, -bwa), 0, 2 * djump, ZCard, XYCard);
		cout << "current y:	" << y << "	of	" << scan.y_n << endl;
		scan.SaveRow(y);
	}

	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, djump / 4, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//scan.SaveFiles();
	cout << "Scanning done!" << endl;
	getchar(); getchar();
}
void Regulator::VAC_scan() {}

void Regulator::R_NV_TransistorCalibration(double Vg_min , double Vg_max , double incr, string dir) {
	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2, ADC_BUF_SIZE_2);
	int point_num = (Vg_max - Vg_min) / incr;
	vector<double> noise(point_num, point_num);
	vector<double> volts(point_num, point_num);
	string timestr = get_time_string();
	std::filesystem::create_directories(dir + timestr);
	ofstream file;
	file.open(dir + timestr + "/" + "Noise_.dat", std::ofstream::out);
	XYCard.StopReadStream();
	XYCard.StartReadStream();
	for (int i = 0; i < point_num; i++) {

		ZCard.SingleAnalogOut(Vg_min + i * incr);
		XYCard.StopReadStream();
		uwait(600000);
		XYCard.StartReadStream();
		
		data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 500, ADC_BUF_SIZE_2);
		noise[i] = data.Average(ADC_BUF_SIZE_2 / 2, 2);
		volts[i] = data.Average(ADC_BUF_SIZE_2 / 2, 3);
		cout << endl << endl << i << endl;
		cout << " printing data..." << endl;
		file << noise[i] << "   " << volts[i] /*Vg_min + i * incr*/ << endl;
		cout << "data printed in file Noise_.dat " << endl;

	}
}
void Regulator::R_V_TransistorCalibration(double incr, double Vg_min, double Vg_max, double Vsd_crit, int delay_us, string dir) {
	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2, ADC_BUF_SIZE_2);
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	ZCard.SingleAnalogOut(0.0, Z_OUT_FINE);
	string timestr = get_time_string();
	double Vg = Vg_min, Vr =  0, Vg_m = 0, Vsd = 0;
	std::filesystem::create_directories(dir + timestr);
	ofstream file;
	XYCard.StopReadStream();
	XYCard.StartReadStream();

	for (double Vg = Vg_min; Vg < Vg_max; Vg += incr) {
		file.open("../../scans/" + timestr + "/" + "VAC_Vg_" + to_string(Vg) + ".dat", std::ofstream::out);
		Vr = 0;
		ZCard.SingleAnalogOut(Vg, Z_OUT);
		ZCard.SingleAnalogOut(Vr, Z_OUT_FINE);
		Vsd = 0;
		uwait(delay_us);
		while ((Vsd < Vsd_crit) && (Vr <= 1)) {


			ZCard.SingleAnalogOut(Vr, Z_OUT_FINE);
			XYCard.StopReadStream();
			uwait(delay_us);
			XYCard.StartReadStream();

			data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 500, ADC_BUF_SIZE_2);
			Vsd = data.Average(ADC_BUF_SIZE_2 / 4, 2);
			Vg_m = data.Average(ADC_BUF_SIZE_2 / 4, 3);

			file << Vsd << "   " << Vr << "   " << Vg_m << endl;
			if (Vg > 0.45) Vr += 0.2;
			else if (Vg > 0.4) Vr += 0.1;
			else if (Vg > 0.38) Vr += 0.02;
			else if (Vg > 0.36) Vr += 0.0025;
			else Vr += 10 * MIN_STEP_SIZE;

		}
		while ((Vr >= -1) && (Vsd > -Vsd_crit)) {


			ZCard.SingleAnalogOut(Vr, Z_OUT_FINE);
			XYCard.StopReadStream();
			uwait(delay_us);
			XYCard.StartReadStream();

			data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 500, ADC_BUF_SIZE_2);
			Vsd = data.Average(ADC_BUF_SIZE_2 / 4, 2);
			Vg_m = data.Average(ADC_BUF_SIZE_2 / 4, 3);

			file << Vsd << "   " << Vr << "   " << Vg_m << endl;
			if (Vg > 0.45) Vr -= 0.2;
			else if (Vg > 0.4) Vr -= 0.1;
			else if (Vg > 0.38) Vr -= 0.02;
			else if (Vg > 0.36) Vr -= 0.0025;
			else Vr -= 10 * MIN_STEP_SIZE;
		}
		while ((Vr <= 0)) {


			ZCard.SingleAnalogOut(Vr, Z_OUT_FINE);
			XYCard.StopReadStream();
			uwait(delay_us);
			XYCard.StartReadStream();

			data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 500, ADC_BUF_SIZE_2);
			Vsd = data.Average(ADC_BUF_SIZE_2 / 4, 2);
			Vg_m = data.Average(ADC_BUF_SIZE_2 / 4, 3);

			file << Vsd << "   " << Vr << "   " << Vg_m << endl;
			if (Vg > 0.45) Vr += 0.2;
			else if (Vg > 0.4) Vr += 0.1;
			else if (Vg > 0.38) Vr += 0.02;
			else if (Vg > 0.36) Vr += 0.0025;
			else Vr += 10*MIN_STEP_SIZE;
		}
		cout << endl << endl << Vg << endl;
		cout << " printing data..." << endl;
		cout << "data printed in file VAC_Vg_" << Vg << ".dat " << endl;
		file.close();
	}

}