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


void Regulator::wait_clear_buf(Card& card, int cnt ) {
	for (int i = 0; i < cnt; i++)  card.SingleAnalogRead();
}

void Regulator::Step(int axis, int dir, double step_size ) {
	Vecter step;
	if (axis == X_)
		step = Vecter(step_size, 0, 0);
	if (axis == Y_)
		step = Vecter(0, step_size, 0);
	std::cout << axis << endl;
	if (dir > 0) {
		piezo.MoveTo(step, 0, 5 * MIN_STEP_SIZE, ZCard, XYCard);
		JHome();
	}
	else if (dir < 0) {
		piezo.JumpTo(step, ZCard, XYCard);
		MHome(5 * MIN_STEP_SIZE);
	}
}


Regulator::Regulator(double i_offset, double noise_limit_V, double frequency , double bias) :
	XYCard(1, 4, ADC_BUF_SIZE_2),
	ZCard(),
	noise_limit_V(noise_limit_V),
	frequency(frequency),
	current_offset(i_offset), bias(bias) {
	ZCard.SingleAnalogOut(bias, BIAS_OUT);
	//ZCard.SingleAnalogRead();
	//std::cout <<"Current signul: "<< ZCard.data.average<<endl;

}
Regulator::~Regulator() {
	MHome();
	ZCard.~NICard(); 
	XYCard.FullStop();
	XYCard.~LCard();
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

void Regulator::ZStep(int dir, double step_size, bool makelogs, string folder) {
	if (dir > 0) {
		for (double i = 0; i < step_size; i += MIN_STEP_SIZE) {
			ZCard.SingleAnalogOut(i);
		}
		ZCard.SingleAnalogOut(0,0);
	}
	else {
		for (double i = step_size; i > 0; i -= MIN_STEP_SIZE) {
			ZCard.SingleAnalogOut(i);
		}

	}
	if (makelogs) { 
		make_logs(folder, "Z step " + to_string(dir ? step_size : -step_size)); 
	}
}
void Regulator::StepXY(int x_steps, int y_steps, double step_size, string folder) {
	for (int i = 0; i < abs(x_steps); i++) Step(X_, x_steps, step_size);
	for (int i = 0; i < abs(y_steps); i++) Step(Y_, y_steps, step_size);
	make_logs(folder, "X steps done: " + to_string(x_steps) + "\n Y steps done: " + to_string(y_steps));
}



void Regulator::Retract(int steps, double step_incr, double rpt, string folder) {
	int step = -3;
	MHome();
	double height = rise();
	MHome();
	double st_sz = max(height / 1.25 - 0.2, 0.4);
	
	for (double sz = st_sz; sz <= 5; sz += step_incr) {
		for (int i = 0; i < rpt; i++) {
			ZCard.SingleAnalogOut(5);
			ZStep(-1, sz, false);
			step += 1;
			//uwait(100000);
		}
		//std::cout << "backstep " << sz << endl;

	}
	while (step < steps) {
		ZStep(-1, 5, false);
		step++;
	}
	make_logs(folder, "Rectracted steps: "+ to_string(steps));
}
double Regulator::rise(double bias_, double bwa, double target_V, double djump, string folder) {
	bias = bias_;

	/*Обнуляем всё*/
	bool is_touch = false;
	double last_height = 0;
	
	/*Касание и выдержка*/

	for (int i = 0; i < 10; i++) {//считывает n раз для очистки буфера
		ZCard.SingleAnalogRead();
		is_touch = (abs(ZCard.data_[0]) > target_V);

	}

	while (!is_touch) {

		piezo.ZJump(djump, ZCard);
		last_height = piezo.Position('Z');

		is_touch = (abs(ZCard.SingleAnalogRead()) > target_V);

		if (last_height < 0.05) is_touch = false;
		if (piezo.Position('Z') >= 5) {
			//uwait(delay_micro);
			MHome();
			break;
		}
	}

	
	piezo.Move(Vecter(0, 0, -bwa), 0, 5 * MIN_STEP_SIZE, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//uwait(1000000);
	cout << "Height: " << last_height << endl;
	make_logs(folder, " Check rise | Touch height: " + to_string(last_height));
	return last_height;
}
double Regulator::Landing(double bias_, double range, double target_V, double delay_micro, double djump, string folder) {
	make_logs(folder, "Landing started");
	bias = bias_;
	target_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	bool is_touch = false;
	int stp_count = 0;
	double last_height = 99;
	Vecter zero(0, 0, 0);
	for (int i = 0; i < 10; i++) {//считывает n раз для очистки буфера
		ZCard.SingleAnalogRead();
		is_touch = (abs(ZCard.SingleAnalogRead()) > target_V);
		//std::cout << ZCard.data.Average(8, 0) << endl;
	}
	//getchar(); getchar();
	while (!is_touch) {

		piezo.ZJump(djump, ZCard);
		last_height = piezo.Position('Z');
		//stp_count++;
		//uwait(delay_micro);

		is_touch = (abs(ZCard.SingleAnalogRead()) > target_V);

		if (last_height < 0.2) is_touch = false;
		if (piezo.Position('Z') >= range) {
			//uwait(delay_micro);
			stp_count++;
			piezo.ZJumpTo(0, ZCard);
			for (int i = 0; i < 20; i++) {
				uwait(1000);
				ZCard.SingleAnalogRead();
			}


		}
	}

	
	piezo.Move(Vecter(0, 0, -0.2), delay_micro, 8*djump, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	piezo.JumpTo(Vecter(0, 0, 0), ZCard, XYCard);
	std::cout << "Touch current signal: " << ZCard.data_[0] << endl;
	std::cout << "Touch_height: " << last_height << endl;
	std::cout << "Landing done" << endl;
	std::cout << "Steps:" << stp_count << endl;
	//getchar(); getchar();
	ZCard.StopReadStream();
	make_logs(folder, "Landing completed\nSteps done:"+ to_string(stp_count)+ "\n Touch height: "+ to_string(last_height));
	return last_height;
}
void Regulator::FullCalibration(int points_num, ofstream file, string filename) {
	///not realized
}




void Regulator::IntPID(double bias_, double target_V, double duration_us, double pid_log_offset, double start_offset, string folder ) {
	//int i = 0;
	//piezo.Move(Vecter(0, 0, Landing()-0.01), 0, djump, ZCard, XYCard);
	//piezo.Move(Vecter(0, 0, 0), 0, djump, ZCard, XYCard);
	bias = bias_;
	target_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	ZCard.SingleAnalogRead();

	Timer tmr;
	if (duration_us == 0) {
		while (true) {
			ZCard.SingleAnalogRead();
			piezo.ZJumpTo(pid.signal(log(pid_log_offset - target_V), log(pid_log_offset - ZCard.data_[0]), tmr.get_loop_interval()), ZCard);
		}
	}
	else {
		pid.set_zero_pos(piezo.Position(), start_offset);
		while (tmr.get_full_interval() <= duration_us) {
			ZCard.SingleAnalogRead();
			piezo.ZJumpTo(pid.signal(log(pid_log_offset - target_V), log(pid_log_offset - ZCard.data_[0]), tmr.get_loop_interval()), ZCard);
		}
	}



}
double Regulator::IntPID_exp(double bias_, double target_V, double duration_us, double start_pos, int polarity, double I_to_nA, double touch_lim, string folder) {

	bias = bias_;
	target_V += current_offset;
	//make_logs(folder, "Experimental PID started: \nParameters: \n	Bias: " + to_string(bias_) + "\n	Target_V: " + to_string(target_V));
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
			piezo.ZFJumpTo(pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval()), ZCard);
		}
	}
	else {
		pid.set_zero_pos(start_pos);
		tmr.get_loop_interval();
		while (tmr.get_full_interval() <= duration_us) {
			piezo.ZFJumpTo(pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval()), ZCard);
			if (piezo.V_uplimit.z_proj < pid.last_signal)	{
				pid.set_zero_pos(piezo.V_uplimit.z_proj);
			}
			//cout << ZCard.cur_volt[0] <<"	" << ZCard.cur_volt[1] << endl;
		}
	
		
	}
	//make_logs(folder, "Experimental PID stopped ");
	return pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval());

}
void Regulator::ExtPID(double bias_, double delay, double bwa, double crit_V, double slope, double djump, string folder) {}

void Regulator::VANC_PID(int count, double target_V, double bias_,  double pre_wait, double delay, string folder)
{
	bias = bias_;
	target_V += current_offset;
	ADC_Collect data = XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
	Timer tmr;
	string timestr = get_time_string();
	std::cout << "ANC measurements with PID started: " << endl;
	make_logs(folder, "VANC measurements with PID started \nParameters: \n	Bias: " + to_string(bias_) + "\n	Target_V: " + to_string(target_V) + "\n	Count: " + to_string(count) + "\n	Delay: " + to_string(delay));
	double  height = IntPID_exp(bias, target_V, pre_wait * 1000000, 0);

	for (int i = 1; i <= count; i++) {
		tmr.get_loop_interval();
		XYCard.StartReadStream();
		height = IntPID_exp(bias, target_V, delay * 1000000, height);
		data = XYCard.AnalogRead(0, ADC_BUF_SIZE_2);
		XYCard.StopReadStream();
		data.print_f_VANC("VANC_" + to_string(i), ".bin", "../../scans/" + timestr);
		cout << "done   " << i << " of " << count << " VANCS" << endl;
		//cin >> stop;
		//if (stop) break;
		cout << "loop time : " << tmr.get_loop_interval() / 1000000 << endl;
	}
	MHome();
	std::cout << "VANC measurements done !" << endl;
	make_logs(folder, "VANC measurements done ");
}



/////////

VAC Regulator::VAC_(double max, double min, double step, int name, double delay_us, string folder) {
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
VANC Regulator::VANC_(double max, double min, double step, int name, double delay_us, string folder) {
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
void Regulator::TouchScan(double bias_ , double bwa, double crit_V, double x_dim, double y_dim,	double x_step, double y_step, double djump, int up_mult, int pre_wait, string folder) {
	bias = bias_;
	crit_V += current_offset;
	make_logs(folder, "Touch scan started \nParameters: \n	Bias: "+ to_string(bias_)+"\n	BWA: " + to_string(bwa) + "\n	Target_V: " + to_string(crit_V) + "\n	DJump: " + to_string(djump));
	Scan scan(x_dim, y_dim, x_step, y_step);
	/*Обнуляем всё*/
	bool is_touch = false;
	piezo.MoveTo(Vecter(0, 0, 0), 0, djump, ZCard, XYCard);
	/*Касание и выдержка*/

	rise(bias, bwa, crit_V, djump / 2);
	while (!is_touch) {
		piezo.ZJump(djump / 2, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);

		if (piezo.Position() < 2 * bwa) is_touch = 0;
		for (int i = 0; i < 10; i++)  ZCard.SingleAnalogRead();
		ZCard.SingleAnalogRead();
	}
	
	while (is_touch) {

		piezo.ZJump(-2 * djump, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);

		if (piezo.Position() < 2 * bwa) is_touch = 0;
		ZCard.SingleAnalogRead();
	}
	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, djump, ZCard, XYCard);
	uwait(pre_wait * 1000000); // ждём 300 сек

	ZCard.SingleAnalogRead();
	is_touch = (ZCard.SingleAnalogRead() > crit_V);
	while (is_touch) {

		piezo.ZJump(-4 * djump, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);

		if (piezo.Position() < bwa) is_touch = 0;
		ZCard.SingleAnalogRead();
	}
	//piezo.Move(Vecter(0, 0, -bwa), 0, djump, ZCard, XYCard);
	is_touch = (ZCard.SingleAnalogRead() > crit_V);
	cout << "signal : " << ZCard.SingleAnalogRead() << "		Crit_v = " << crit_V << endl;
	cout << "is touch : " << is_touch << endl;
	/*Начало сканирования*/
	for (int y = 0; y < scan.y_n; y++) {
		/*передний ход:*/

		for (int x = 0; x < scan.x_n; x++) {

			uwait(50);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {
				piezo.ZJump(djump, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);

				if (piezo.Position() < bwa) is_touch = 0;

			}

			scan.HFWplot[y][x] = piezo.Position();
			scan.CFWplot[y][x] = ZCard.data_[0] * 1000;

			//uwait(100);

			while (is_touch) {

				piezo.ZJump(-4 * djump, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);
			
				if (piezo.Position() < bwa) is_touch = 0;
				ZCard.SingleAnalogRead();
			}
			piezo.Move(Vecter(0, 0, -bwa), 0, 4 * djump, ZCard, XYCard);
			piezo.Move(Vecter(x_step, 0,  0), 0, 2 * djump, ZCard, XYCard);
		}
		/*задний ход:*/
		for (int x = scan.x_n - 1; x >= 0; x--) {
			
			uwait(50);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);

			
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {

				piezo.ZJump(djump, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);

				if (piezo.Position() < bwa) is_touch = 0;

			}

			scan.HBWplot[y][x] = piezo.Position();
			scan.CBWplot[y][x] = ZCard.data_[0] * 1000;

			//uwait(100);

			while (is_touch) {

				piezo.ZJump(-4 * djump, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);

				if (piezo.Position() < bwa) is_touch = 0;
				ZCard.SingleAnalogRead();
			}
			piezo.Move(Vecter(0, 0, -bwa), 0, 4 * djump, ZCard, XYCard);
			piezo.Move(Vecter(-x_step, 0, 0), 0, 2 * djump, ZCard, XYCard);
		}

		piezo.Move(Vecter(0, y_step, 0), 0,  djump, ZCard, XYCard);
		std::cout << "current y:	" << y << "	of	" << scan.y_n << endl;
		scan.SaveRow(y);
	}

	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, djump / 4, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//scan.SaveFiles();
	std::cout << "Scanning done!" << endl;
	make_logs(folder, "Touch scan done ");
	//getchar(); getchar();
}
void Regulator::ConstH_Scan(double bias_, double target_V, double pid_delay, double point_delay, double x_dim, double y_dim, double x_step, double y_step, double djump, double bwa, int pre_wait, string folder)
{
	
	bias = bias_;
	target_V += current_offset;
	make_logs(folder, "Const height scan started \nParameters: \n	Bias: " + to_string(bias_) + "\n	Target_V: " + to_string(target_V) + "\n	DJump: " + to_string(djump));
	Scan scan(x_dim, y_dim, x_step, y_step);
	/*Обнуляем всё*/
	bool is_touch = false;
	piezo.MoveTo(Vecter(0, 0, 0), 0, djump, ZCard, XYCard);
	/*Касание и выдержка*/

	rise(bias, bwa, target_V);
	
	double  height = IntPID_exp(bias, target_V, pre_wait * 1000000, 0,1);
	Vecter start_pos(0, 0, height - bwa);
	is_touch = (abs(ZCard.SingleAnalogRead()) > target_V);
	cout << "signal : " << ZCard.SingleAnalogRead() << "		Crit_v = " << target_V << endl;
	cout << "is touch : " << is_touch << endl;
	/*Начало сканирования*/
	for (int y = 0; y < scan.y_n; y++) {

		height = IntPID_exp(bias, target_V, pid_delay * 1000000, start_pos.z_proj);
		piezo.Move(Vecter(0, y_step * y, 0), 0, djump, ZCard, XYCard);
		start_pos.set(0, 0, height - bwa);
		/*передний ход:*/
		for (int x = 0; x < scan.x_n; x++) {
			piezo.Move(Vecter(x_step, 0, 0), 0, 2 * djump, ZCard, XYCard);
			uwait(point_delay);
			ZCard.SingleAnalogRead();
			scan.HFWplot[y][x] = piezo.Position();
			scan.CFWplot[y][x] = ZCard.data_[0];
			
		}
		/*задний ход:*/
		for (int x = scan.x_n - 1; x >= 0; x--) {
			piezo.Move(Vecter(-x_step, 0, 0), 0, 2 * djump, ZCard, XYCard);
			uwait(point_delay);
			ZCard.SingleAnalogRead();
			scan.HBWplot[y][x] = piezo.Position();
			scan.CBWplot[y][x] = ZCard.data_[0];
		
		}

		piezo.MoveTo(start_pos, 0, djump, ZCard, XYCard);
		uwait(point_delay);
		std::cout << "current y:	" << y << "	of	" << scan.y_n << endl;
		scan.SaveRow(y);
	}

	piezo.Move(Vecter(0, 0, -bwa), 0, djump / 4, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//scan.SaveFiles();
	std::cout << "Scanning done!" << endl;
	make_logs(folder, "Constant height scan done ");
	//getchar(); getchar();
}
void Regulator::VAC_scan() {}

void Regulator::Pn_CVg_TransistorCalibration(double Vg_min , double Vg_max , double incr, int delay, string folder) {
	std::cout << endl << "Pn_CVg_TransistorCalibration started..." << endl;
	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2000, ADC_BUF_SIZE_2);
	int point_num = (Vg_max - Vg_min) / incr;
	double dispersion = 0;
	double max_delta = 0;
	
	vector<double> noise(point_num, point_num);
	vector<double> volts(point_num, point_num);
	string timestr = get_time_string();
	std::filesystem::create_directories(folder + timestr);
	ofstream file, file_Back;
	make_logs(folder, "Pn_CVg_TransistorCalibration started\nTime per point: " + to_string((float)ADC_BUF_SIZE_2/ADC_TGT_FREQ)+" s");
	std::cout << endl << "Output directories created: " << folder + timestr << endl;
	std::cout << endl << point_num << " data points expected:"  << endl;
	std::cout << endl << " Expexted time:" << (delay + ADC_BUF_SIZE_2 / 2) * point_num * 2 / 1000000 / 60 << "m" << endl;
	ZCard.SingleAnalogOut(0.0, Z_OUT_FINE);
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	uwait(1000000);
	file.open(folder + timestr + "/" + "Noise_D.dat", std::ofstream::out);
	file_Back.open(folder + timestr + "/" + "Noise_B.dat", std::ofstream::out);
	XYCard.StopReadStream();
	XYCard.StartReadStream();
	for (int i = 0; i < point_num; i++) {

		ZCard.SingleAnalogOut(Vg_min + i * incr, Z_OUT);
		XYCard.StopReadStream();
		XYCard.StartReadStream();
		uwait(delay);
		
		
		data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2000, ADC_BUF_SIZE_2);
		noise[i] = data.Average(ADC_BUF_SIZE_2 / 2, NOISE_CH);
		volts[i] = Vg_min + i * incr;

		std::cout << endl << endl <<  i + 1 << "  of  " << point_num << "  FW points done " << endl;
		file << noise[i] << "   " << volts[i] <<endl;
		
	}
	std::cout << "data printed in file Noise_D.dat " << endl;
	for (int i = point_num-1; i >=0; i--) {

		ZCard.SingleAnalogOut(Vg_min + i * incr, Z_OUT);
		XYCard.StopReadStream();
		uwait(delay);
		XYCard.StartReadStream();

		data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2000, ADC_BUF_SIZE_2);
		noise[i] = data.Average(ADC_BUF_SIZE_2 / 2, NOISE_CH);
		volts[i] = Vg_min + i * incr;

		std::cout << endl << endl << point_num - i << "  of  " << point_num << "  BW points done " << endl;
		file_Back << noise[i] << "   " << volts[i]  << endl;
	
	}
	std::cout << "data printed in file Noise_B.dat " << endl;
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	std::cout << endl << "Calibration completed!" << endl;
	make_logs(folder, "Pn_CVg_TransistorCalibration completed");
}
void Regulator::R_CVg_TransistorCalibration(double incr, double Vg_min, double Vg_max, double Vsd_crit, double Vbias_crit, int delay_us, string folder) {
	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 500, ADC_BUF_SIZE_2);
	std::cout << endl << " R_CVg_TransistorCalibration started..." << endl;
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	ZCard.SingleAnalogOut(0.0, Z_OUT_FINE);
	string timestr = get_time_string();
	double Vg = Vg_min, Vbias =  0, Vsd = 0;
	double noise;
	double offset = 0.004;
	std::filesystem::create_directories(folder + timestr);
	ofstream file;
	std::cout << endl << " Output directories created..." << endl;
	make_logs(folder, "R_CVg_TransistorCalibration started\nTime per point: " + to_string((float)ADC_BUF_SIZE_2 / ADC_TGT_FREQ) + " s");
	XYCard.StopReadStream();
	XYCard.StartReadStream();
	int count = 0;
	int dir = -1;
	for (double Vg = Vg_min; Vg < Vg_max; Vg += incr) {
		count++;
		file.open("../../scans/" + timestr + "/" + "VAC_Vg_" + to_string(Vg) + ".dat", std::ofstream::out);
		Vbias = 0;
		ZCard.SingleAnalogOut(Vg, Z_OUT);
		ZCard.SingleAnalogOut(Vbias, Z_OUT_FINE);

		dir = -1;
		uwait(delay_us);
		while ((dir < 1) || (Vbias <= MIN_STEP_SIZE)) {	
			
			ZCard.SingleAnalogOut(Vbias- 0.0013 , Z_OUT_FINE);
			XYCard.StopReadStream();
			XYCard.StartReadStream();
			uwait(delay_us);
			

			data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2000, ADC_BUF_SIZE_2);
			Vsd = data.Average(ADC_BUF_SIZE_2 / 2, R_CALIBR_CH);
			noise = data.Average(ADC_BUF_SIZE_2 / 2, NOISE_CH);

			file << Vsd << "   " << Vbias << "   " << noise << "   " << Vg << endl;
			
			if (((Vsd >= Vsd_crit- offset) && (dir == -1)) ||
				((Vsd <= -Vsd_crit - offset) && (dir == 0)) ||
				(Vbias >= Vbias_crit ) ||
				(Vbias <= -Vbias_crit)) {
				
				dir++;
				cout << dir << endl;
			}
			
			Vbias += TrBiasStepper(Vg, dir ? FORWARD : BACKWARD);
			
		}
	

		std::cout << " printing data..." << endl;
		std::cout << " data printed in file VAC_Vg_" << Vg << ".dat " << endl;
		std::cout << count << "  of  " << (Vg_max - Vg_min) / incr << " VACs done" << endl;
		file.close();
	}
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	ZCard.SingleAnalogOut(0, Z_OUT_FINE);
	std::cout << endl << "Calibration completed!" << endl;
	make_logs(folder, "R_CVg_TransistorCalibration completed");
}







