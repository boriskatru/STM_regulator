#include "regulator.h"

PID::PID(double r_,double bias_,double P_, double I_, double D_ ) :
	r(r_),
	P(P_),
	I(I_),
	D(D_), 
	err(0), d_err(0), diff_count(10),target(0.1),
	integral(0),last_signal(0),tmp(0),bias(bias_) {
	save_settings();

}
PID::~PID() {
	P = I = D = 0;
}
void PID::set_zero_pos(double pos, double offset ) {
	integral = (pos + offset) / I;
}
double PID::signal(double y, double dtime,  double max_step, double max_err ) {
	err = y - r;
	if (abs(err) > max_err) err = sign(err) * max_err;
	integral += err * dtime;
	if (integral <= 0)  integral = 0;
	if (dtime > 0)	d_err = ((diff_count - 1) * d_err + err / dtime) / diff_count;
	else d_err = 0;
	tmp = P * err + I * integral + D * d_err;
	/*if ((tmp - last_signal) > max_step) {
		last_signal += max_step;
	}
	else if ((tmp - last_signal) < max_step) {
		last_signal -= max_step;
	}
	else*/ 
	last_signal = tmp;
	if (isnan(last_signal)) {
		cout << "pid failed/ NAN SIGNAL|Fa hasf asfiaf oaoive uhoa uh" << endl;
		tmp = P * err + I * integral;
	}
	return last_signal;
}

void PID::reset(double target_, double bias_, double P_, double I_, double D_){
	target = target_;
	r = CHTransform(target * I_to_nA, bias);
	bias = bias_;
	P = P_;
	I = I_;
	D = D_;
}

void PID::reset_from_file(string folder){
	ifstream file;
	file.open(folder + "PID_SETTINGS.txt", std::ios::in);	
	file >> target >> bias >> P >> I >> D;	
	r = CHTransform(target * I_to_nA, bias);
	file.close();
}

void PID::save_settings(string folder){
	ofstream file;
	file.open(folder + "PID_SETTINGS.txt", std::ios::out);
	file << target << endl << bias << endl << P << endl << I << endl << D;
	file.close();
}

/////////

void Regulator::wait_clear_buf(Card& card, int cnt ) {
	for (int i = 0; i < cnt; i++)  card.SingleAnalogRead();
}

void Regulator::Step(int axis, int dir, double step_size, double step_speed) {
	Vecter step;
	if (axis == X_)
		step = Vecter(step_size, 0, piezo.Position());
	if (axis == Y_)
		step = Vecter(0, step_size, piezo.Position());
	//std::cout << axis << endl;
	if (dir > 0) {
		piezo.MoveTo(step, 0, step_speed * MIN_STEP_SIZE, ZCard, XYCard);
		piezo.JumpTo(Vecter(0, 0, piezo.Position()), ZCard, XYCard);		
	}
	else if (dir < 0) {			
		piezo.JumpTo(step, ZCard, XYCard);
		piezo.MoveTo(Vecter(0, 0, piezo.Position()), 0, step_speed * MIN_STEP_SIZE, ZCard, XYCard);		
	}
}


Regulator::Regulator(double i_offset, double noise_limit_V, double frequency , double bias, string folder):
	XYCard(1, 6, ADC_BUF_SIZE_2),
	ZCard(),
	noise_limit_V(noise_limit_V),
	frequency(frequency),
	current_offset(i_offset), bias(bias),
	folder(folder) {
	ZCard.SingleAnalogOut(0, Z_OUT);
	ZCard.SingleAnalogOut(0, Z_OUT_FINE);
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


/////////

void Regulator::MoveTo(double step)
{
}

void Regulator::MHome(double step) {
	piezo.MoveTo(Vecter(0, 0, 0), 0, step, ZCard, XYCard);
}
void Regulator::ZHome(double step) {
	piezo.MoveTo(Vecter(piezo.Position('X'), piezo.Position('Y'), 0), 0, step, ZCard, XYCard);
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
int Regulator::GetStatus(string folder)
{
	ifstream file;
	file.open(folder +"/" + "STATUS.txt", std::ios::in);
	int status = 2;
	file >> status;
	file.close();
	return status;
}
bool Regulator::CheckStatus(string stop_message, string folder) {	
	while (GetStatus() == 1) {
		uwait(100000);
	}
	if (GetStatus() == 0) {
		MHome();
		std::cout << stop_message << endl;
		make_logs(folder, stop_message);
		return 1;
	}
	else return 0;
}
//void Regulator::CheckStatus(string message, double& arg,double bwa,  string folder)
//{
//	if (GetStatus() < 2) {
//		piezo.Move(Vecter(0, 0, -0.15), 100, MIN_STEP_SIZE / 2, ZCard, XYCard);
//		arg = arg - 0.15;
//		while (GetStatus() < 2) {
//			if (GetStatus() == 0) {
//				MHome();
//				std::cout << "VANC measurements stopped by user" << endl;
//				make_logs(folder, "VANC measurements stopped by user ");
//				exit(0);
//			}
//			uwait(100000);
//		}
//	}
//}
void Regulator::WriteStatus(int status, string folder)
{
	ofstream file;
	file.open(folder + "/" + "STATUS.txt", std::ios::out);
	file << status;
	file.close();
}
void Regulator::ResetPIDFromFile(string folder)
{
	pid.reset_from_file(folder);
	bias = pid.bias;
	pid.set_zero_pos(pid.last_signal);
}
void Regulator::LoadParamsFromFile(string folder,  string name , int count, double**arr)
{		
		ifstream file;
		file.open(folder + "/" + name, std::ios::in);		
		
		double tmp;		
		for (int i = 0; i < count; i++) {
			file >> tmp;				
			*arr[i] = tmp;
			cout << "Указатель   " << arr[i] << "	" << *arr[i] << endl;
		}
		file.close();
}
void Regulator::SaveParamsToFile(string folder, string name, int count, ...)
{
	va_list vl;
	ofstream file;
	cout << folder + name << endl;
	file.open(folder + name, std::ios::out);
	
	va_start(vl, count);	
	for (int i = 0; i < count; i++) {	
		file << va_arg(vl, double) << endl;		
	}
	va_end(vl);

	file.close();
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
void Regulator::StepXY(int x_steps, int y_steps, bool need_logs, double step_size, double step_speed, double delay, string folder) {
	WriteStatus(2);
	//const int param_num = 8;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &step_V};  ALL TO DOUBLE!!!!!!!!!!!!!!!!!!!!!!	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "StepXY.txt", 6, (double)x_steps, (double)y_steps, (double)need_logs, step_size, step_speed, delay);
	//return;

	for (int i = 0; i < abs(x_steps); i++) { 
		if (abs(ZCard.SingleAnalogRead()) < 0.5) {
			Step(X_, x_steps, step_size, step_speed);
			uwait(delay);
			if (CheckStatus("Steps stopped by user. "+ to_string(i)+" X steps done")) return;
		}
		else { 
			cout << "AHTUNG!!!  OBSTACLE!!! STEPS STOPPED!!!" << endl;
			make_logs(folder, "AHTUNG!!!OBSTACLE!!!STEPS STOPPED!!!");
			return; 
		}
	}
	for (int i = 0; i < abs(y_steps); i++) {
		if (abs(ZCard.SingleAnalogRead()) < 0.5) {
			Step(Y_, y_steps, step_size, step_speed);
			uwait(delay);
			if (CheckStatus("Steps stopped by user. " + to_string(x_steps) + " X steps and" + to_string(i) + " Y steps done")) return;
		}
		else {
			cout << "AHTUNG!!!  OBSTACLE!!! STEPS STOPPED!!!" << endl;
			make_logs(folder, "AHTUNG!!!OBSTACLE!!!STEPS STOPPED!!!");
			return;
		}
	}
	if (need_logs) {
		string log = "";
		if (x_steps != 0) log += "X steps done: " + to_string(x_steps);
		if (y_steps != 0) log += "\n Y steps done: " + to_string(y_steps);
		make_logs(folder, log);
	}
	
}

/////////

void Regulator::Retract(int steps, double touch_v, double step_incr, string folder) {
	WriteStatus(2);
	int step = -2;
	ZHome();
	double height = rise(0.05, 0.2, touch_v, MIN_STEP_SIZE / 10);
	ZHome();
	double st_sz = max(height / 1.25 - 0.2, 0.4);
	
	for (double sz = st_sz; sz <= 5; sz += step_incr) {

		ZCard.SingleAnalogOut(min(5, sz * 1.7));
		ZCard.SingleAnalogOut(sz - 0.25);
		ZStep(-1, sz, false);
		step += 1;
		//uwait(100000);

	//std::cout << "backstep " << sz << endl;

	}
	while (step < steps) {
		ZStep(-1, 5, false);
		step++;
	}
	make_logs(folder, "Rectracted steps: "+ to_string(steps));
	WriteStatus(0);
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
			ZHome(MIN_STEP_SIZE * 2);
			break;
		}
	}

	
	piezo.Move(Vecter(0, 0, -bwa), 0, 5 * MIN_STEP_SIZE, ZCard, XYCard);
	ZHome();
	ZCard.StopReadStream();
	//uwait(1000000);
	cout << "Height: " << last_height << endl;
	make_logs(folder, " Check rise | Touch height: " + to_string(last_height));
	return last_height;
}
double Regulator::Landing(double bias_, double range, double target_V, double delay_micro, double step_speed, string folder) {
	WriteStatus(2);
	//const int param_num = 5;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &step_V};  ALL TO DOUBLE!!!!!!!!!!!!!!!!!!!!!!	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "Landing.txt", 5, bias_, range, target_V, delay_micro, djump);
	//return 5;
	make_logs(folder, "Landing started");
	bias = bias_;
	target_V += current_offset;
	//ZCard.SingleAnalogOut(bias, BIAS_OUT);
	bool is_touch = false;
	int stp_count = 0;
	double last_height = 99;
	double djump = step_speed * MIN_STEP_SIZE;
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
			if (CheckStatus("Landing stopped by user")) return 0;
			
			for (int i = 0; i < 10; i++) {
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
	WriteStatus(0);
	return last_height;
}
void Regulator::FullCalibration(int points_num, ofstream file, string filename) {
	///not realized
}

/////////

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
double Regulator::IntPID_exp(double bias_, double target_V, double duration_us, double start_pos, int polarity,  double touch_lim, int update_delay, string folder) {
	pid.reset_from_file();
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
	int i = 0;
	Timer tmr;
	if (duration_us == -1) {
		pid.set_zero_pos(start_pos);
		while (true) {
			piezo.ZFJumpTo(pid.signal(CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval()), ZCard);
			if (piezo.V_uplimit.z_proj < pid.last_signal) {
				pid.set_zero_pos(piezo.V_uplimit.z_proj);
			}
			i++;
			if (i > update_delay) {
				i = 0;
				ResetPIDFromFile();				
				if (GetStatus() < 2) {
					piezo.Move(Vecter(0, 0, -0.15), 100, MIN_STEP_SIZE / 2, ZCard, XYCard);					
					pid.set_zero_pos(pid.last_signal - 0.15);
					
					while (GetStatus() < 2) {
						if (GetStatus() == 0) {
							ZHome();
							std::cout << "VANC measurements stopped by user" << endl;
							make_logs(folder, "VANC measurements stopped by user ");
							return 0;
						}
						uwait(100000);
					}
				}
				tmr.get_loop_interval();
			}
		}
	}
	else {
		pid.set_zero_pos(start_pos);			
		tmr.get_loop_interval();
		while (tmr.get_full_interval() <= duration_us) {
			piezo.ZFJumpTo(pid.signal(CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval()), ZCard);
			if (piezo.V_uplimit.z_proj < pid.last_signal)	{
				pid.set_zero_pos(piezo.V_uplimit.z_proj);
			}
			i++;
			if (i > update_delay) {
				i = 0;
				ResetPIDFromFile();				
				if (GetStatus() < 2) {
					piezo.Move(Vecter(0, 0, -0.15), 100, MIN_STEP_SIZE / 2, ZCard, XYCard);
					pid.set_zero_pos(pid.last_signal - 0.15);
					while (GetStatus() < 2) {
						if (GetStatus() == 0) {
							ZHome();
							std::cout << "VANC measurements stopped by user" << endl;
							make_logs(folder, "VANC measurements stopped by user ");
							return 0;
						}
						uwait(100000);
					}
				}
				tmr.get_loop_interval();
			}
			//cout << ZCard.cur_volt[0] <<"	" << ZCard.cur_volt[1] << endl;
		}
	
		
	}
	//make_logs(folder, "Experimental PID stopped ");
	return pid.signal(CHTransform(target_V * I_to_nA, bias), CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, bias), tmr.get_loop_interval());

}
void Regulator::ExtPID(double bias_, double delay, double bwa, double crit_V, double slope, double djump, string folder) {}

/////////

void Regulator::VANC_PID(int count, double target_V, double bias_,  double pre_wait, double delay, string folder)
{
	WriteStatus(2);

	bias = bias_;
	target_V += current_offset;
	ADC_Collect data = XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
	Timer tmr;
	string timestr = get_time_string();
	std::cout << "ANC measurements with PID started: " << endl;
	make_logs(folder, "VANC measurements with PID started \nParameters: \n	Bias: " + to_string(bias_) + "\n	Target_V: " + to_string(target_V) + "\n	Count: " + to_string(count) + "\n	Delay: " + to_string(delay));
	
	pid.reset(target_V, bias);
	pid.save_settings();
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
		if(GetStatus() < 2) { 
			piezo.Move(Vecter(0, 0, -0.15), 100, MIN_STEP_SIZE / 2, ZCard, XYCard);
			height = height - 0.15;
			while (GetStatus() < 2) {
				if (GetStatus() == 0) {
					ZHome();
					std::cout << "VANC measurements stopped by user" << endl;
					make_logs(folder, "VANC measurements stopped by user ");
					return;
				}
				uwait(100000);
			}
		}

	}
	ZHome();
	std::cout << "VANC measurements done !" << endl;
	make_logs(folder, "VANC measurements done ");
	WriteStatus(0);
}
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

/////////

void Regulator::CapStepScan(double bias_, double freq, double crit_V, int x_steps, int y_steps, int X_step_sz, int Y_step_sz, double step_V, string folder) {
	WriteStatus(2);
	//const int param_num = 8;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &step_V};  ALL TO DOUBLE!!!!!!!!!!!!!!!!!!!!!!	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "CapStepScan.txt", 8, bias_, freq, crit_V, (double)x_steps, (double)y_steps, (double)X_step_sz, (double)Y_step_sz, step_V);
	//return;
	string log = "Capasitance step scan started \nParameters: \n	Bias: " + to_string(bias_) +
		"\n	X Step size: " + to_string(X_step_sz) +
		"\n	Y Step size: " + to_string(Y_step_sz) +
		"\n	X size: " + to_string(x_steps) +
		"\n	Y size:" + to_string(y_steps);
	make_logs(folder, log);
	bias = bias_;
	crit_V += current_offset;
	
	Scan scan_fw(x_steps, y_steps, 1,1);
	//Scan scan_bw(x_steps*X_FW_BW, y_steps, 1, 1);
	/*Обнуляем всё*/
	
	cout << "signal : " << ZCard.SingleAnalogRead() << endl;
	
	int fwx_steps = 0;
	int bwx_steps = 0;
	/*Начало сканирования*/
	for (int y = 0; y < scan_fw.y_n; y++) {
		/*передний ход:*/

		for (int x = 0; x < scan_fw.x_n; x++) {

			uwait(50000);	
			scan_fw.HFWplot[y][x] = piezo.Position();
			scan_fw.CFWplot[y][x] = ZCard.SingleAnalogRead();
			if (scan_fw.CFWplot[y][x] > crit_V) {
				StepXY(-X_step_sz, -Y_step_sz, false, step_V);
				cout << "!!!WARNING!!!" << endl << "Obstacle detected!" << endl;
				make_logs(folder, "!!!WARNING!!!  Obstacle detected! Capasitance step scan stopped" );
				return;
			}
			StepXY(X_step_sz, 0, false, step_V);
			if (CheckStatus("Capasitance step scan stopped by user")) return;
		}
		
		/*задний ход:*/	
		fwx_steps += x_steps * X_step_sz;
		int bwx_steps_need = fwx_steps * ((X_step_sz > 0) ? X_FW_BW : 1 / X_FW_BW) - bwx_steps;
		StepXY(-bwx_steps_need, 0, false, step_V);		// чтобы не иметь ошибку на округлении !КАЖЕТСЯ ПРАВИЛЬНЫЙ ЗНАК (-tmp)!
		bwx_steps += bwx_steps_need;

		/*ход по Y:*/
		StepXY(0, Y_step_sz, false, step_V); // если закоментированно, то не едет по Y!!
		std::cout << "current y:	" << y << "	of	" << scan_fw.y_n << endl;
		scan_fw.SaveRow(y);
		//scan_bw.SaveRow(y);
	}
	
	StepXY(0, -Y_step_sz * ((Y_step_sz >= 0) ? Y_FW_BW : 1 / Y_FW_BW) * y_steps, false, step_V); // если закоментированно, то не возвращается по Y!!
	ZCard.StopReadStream();
	//scan.SaveFiles();
	std::cout << "Scanning done!" << endl;
	make_logs(folder, "Capasitance step scan done ");
	WriteStatus(0);
	MHome();
	//getchar(); getchar();
}
void Regulator::CapScan(double bias_, double freq, double crit_V, double point_delay, double x_start, double x_step, double x_stop, double y_start, double y_step, double y_stop, double Z_height, double djump, string folder)
{
	//const int param_num = 12;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &point_delay, &x_start, &x_step, &x_stop, &y_start, &y_step, &y_stop, &Z_height, &djump };	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "CapScan.txt", 12, bias_, freq, crit_V, point_delay, x_start, x_step, x_stop, y_start, y_step, y_stop, Z_height, djump );
	//return;
	WriteStatus(2);
	bias = bias_;
	crit_V += current_offset;
	string log = "Capasitance scan started \nParameters: \n	Bias: " + to_string(bias_) +
		"\n	DJump: " + to_string(djump) +
		"\n	X: " + to_string(x_start) + " : " + to_string(x_step) + " : " + to_string(x_stop) +
		"\n	Y: " + to_string(y_start) + " : " + to_string(y_step) + " : " + to_string(y_stop);
	make_logs(folder, log);
	Scan scan(x_stop - x_start, y_stop - y_start, x_step, y_step);
	/*Обнуляем всё*/
	bool is_touch = false;
	
	piezo.MoveTo(Vecter(x_start, y_start, Z_height), 0, djump / 20, ZCard, XYCard);

	ZCard.SingleAnalogRead();
	is_touch = (ZCard.SingleAnalogRead() > crit_V);
	while (is_touch) {
		piezo.ZJump(-2 * djump, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);		
	}
	//piezo.Move(Vecter(0, 0, -bwa), 0, djump, ZCard, XYCard);
	is_touch = (ZCard.SingleAnalogRead() > crit_V);
	cout << "signal : " << ZCard.SingleAnalogRead() << "		Crit_v = " << crit_V << endl;
	cout << "is touch : " << is_touch << endl;
	/*Начало сканирования*/
	for (int y = 0; y < scan.y_n; y++) {
		/*передний ход:*/

		for (int x = 0; x < scan.x_n; x++) {

			uwait(point_delay);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);
			if (is_touch) {
				MHome();
				cout << "!!!WARNING!!!" << endl << "Obstacle detected!" << endl;
				make_logs(folder, "!!!WARNING!!!  Obstacle detected! Capasitance scan stopped");
				return;
			}
			scan.HFWplot[y][x] = piezo.Position();
			scan.CFWplot[y][x] = ZCard.data_[0] ;				
			piezo.Move(Vecter(x_step, 0, 0), 0, djump, ZCard, XYCard);
		}
		/*задний ход:*/
		for (int x = scan.x_n - 1; x >= 0; x--) {

			uwait(point_delay);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);
			if (is_touch) {
				MHome();
				cout << "!!!WARNING!!!" << endl << "Obstacle detected!" << endl;
				make_logs(folder, "!!!WARNING!!!  Obstacle detected! Capasitance scan stopped");
				return;
			}
			scan.HFWplot[y][x] = piezo.Position();
			scan.CFWplot[y][x] = ZCard.data_[0];
			piezo.Move(Vecter(-x_step, 0, 0), 0, djump, ZCard, XYCard);
		}

		piezo.Move(Vecter(0, y_step, 0), 0, djump, ZCard, XYCard);
		std::cout << "current y:	" << y << "	of	" << scan.y_n << endl;
		scan.SaveRow(y);
		if (GetStatus() < 2) {
			piezo.Move(Vecter(0, 0, -0.3), 100, djump / 4, ZCard, XYCard);
			if (CheckStatus("Capasitance scan stopped by user")) return;
		}
	}
	
	MHome();
	ZCard.StopReadStream();	
	std::cout << "Scanning done!" << endl;
	make_logs(folder, "Capasitance scan done ");
	WriteStatus(0);
}
void Regulator::TouchScan(double bias_, double bwa, double crit_V, double x_start, double x_step, double x_stop,
								double y_start, double y_step, double y_stop, double h_diff_lim, double djump, double up_mult, double pre_wait,  string folder) {
	WriteStatus(2);
	//const int param_num = 13;
	//double* tmp[param_num] = { &bias_, &bwa, &crit_V, &x_start, &x_step, &x_stop, &y_start, &y_step, &y_stop, &h_diff_lim, &djump , &up_mult, &pre_wait };	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "TouchScan.txt", 13, bias_, bwa, crit_V, x_start, x_step, x_stop, y_start, y_step, y_stop, h_diff_lim, djump, (double)up_mult, (double)pre_wait);
	//return;
	string log = "Touch scan started \nParameters: \n	Bias: " + to_string(bias_) +
		"\n	BWA: " + to_string(bwa) + "\n	Target_V: " + to_string(crit_V) +
		"\n	DJump: " + to_string(djump) +
		"\n	X: " + to_string(x_start) + " : " + to_string(x_step) + " : " + to_string(x_stop) +
		"\n	Y: " + to_string(y_start) + " : " + to_string(y_step) + " : " + to_string(y_stop);
	make_logs(folder, log);

	double h_lim = 4.9;
	bias = bias_;
	crit_V += current_offset;	
	Scan scan(abs(x_stop - x_start), abs(y_stop - y_start), abs(x_step), abs(y_step));
	/*Обнуляем всё*/
	bool is_touch = false;
	piezo.MoveTo(Vecter(x_start, y_start, 0), 0, djump / 20, ZCard, XYCard);
	/*Касание и выдержка*/

	if (rise(bias, bwa, crit_V, djump / 2) >= 5) {
		cout << "No touch signal" << endl;
		return;
	}

	piezo.MoveTo(Vecter(x_start, y_start, 0), 0, djump / 20, ZCard, XYCard);

	while (!is_touch) {
		piezo.ZJump(djump / 2, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);

		if (piezo.Position() < 2 * bwa) is_touch = 0;
		for (int i = 0; i < 10; i++)  ZCard.SingleAnalogRead();
		ZCard.SingleAnalogRead();
		if (CheckStatus("Touch scan stopped by user")) return;

	}
	
	while (is_touch) {
		piezo.ZJump(-2 * djump, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);

		if (piezo.Position() < 2 * bwa) is_touch = 0;
		ZCard.SingleAnalogRead();
	}

	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, djump, ZCard, XYCard);

	
	for (int i = 0; i < pre_wait; i++) {		
		if (CheckStatus("Touch scan stopped by user")) return; // ждём pre_wait секунд		
	}

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
				if (piezo.Position() > min((h_lim + X_PLANE_TG * x_step * x + Y_PLANE_TG * y_step * y),4.999)) is_touch = 1;
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
			if (x == 0 && y == 0) h_lim = scan.HFWplot[y][x]+ h_diff_lim;
		}
		/*задний ход:*/
		for (int x = scan.x_n - 1; x >= 0; x--) {
			
			uwait(50);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);

			
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {

				piezo.ZJump(djump, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);				
				if (piezo.Position() > min((h_lim + X_PLANE_TG * x_step * x + Y_PLANE_TG * y_step * y), 4.999)) is_touch = 1;
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
		if (GetStatus() < 2) {
			piezo.Move(Vecter(0, 0, -up_mult * bwa), 100, djump / 4, ZCard, XYCard);
			if (CheckStatus("Touch scan stopped by user")) return;
		}
	}

	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, djump / 4, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//scan.SaveFiles();
	std::cout << "Scanning done!" << endl;
	make_logs(folder, "Touch scan done ");
	WriteStatus(0);
	//getchar(); getchar();
}
void Regulator::ConstHScan(double bias_, double target_V, double pid_delay, double point_delay, double x_start, double x_step, double x_stop,
								double y_start, double y_step, double y_stop, double djump, double bwa, int pre_wait, string folder)
{
	WriteStatus(2);
	//const int param_num = 13;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &step_V};  ALL TO DOUBLE!!!!!!!!!!!!!!!!!!!!!!	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "ConstHScan.txt", 13, bias_, target_V, pid_delay, point_delay, x_start, x_step, x_stop, y_start, y_step, y_stop, djump, bwa, (double)pre_wait);
	//return;
	string log = "Const height scan started \nParameters: \n	Bias: " + to_string(bias_) +
		"\n	Target_V: " + to_string(target_V) + "\n	DJump: " + to_string(djump) +
		"\n	X: " + to_string(x_start) + " : " + to_string(x_step) + " : " + to_string(x_stop) +
		"\n	Y: " + to_string(y_start) + " : " + to_string(y_step) + " : " + to_string(y_stop);
	make_logs(folder, log);


	bias = bias_;
	target_V += current_offset;
	Scan scan(abs(x_stop - x_start), abs(y_stop - y_start), abs(x_step), abs(y_step));
	/*Обнуляем всё*/
	bool is_touch = false;
	piezo.MoveTo(Vecter(x_start, y_start, 0), 0, djump / 20, ZCard, XYCard);
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
		while (GetStatus() < 2) {
			piezo.Move(Vecter(0, 0, -2 * bwa), 100, djump / 4, ZCard, XYCard);
			height = height - 2 * bwa;
			if (CheckStatus("Const height scan stopped by user")) return;
		}
	}

	piezo.Move(Vecter(0, 0, -bwa), 0, djump / 4, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//scan.SaveFiles();
	std::cout << "Scanning done!" << endl;
	make_logs(folder, "Constant height scan done ");
	WriteStatus(0);
	//getchar(); getchar();
}
void Regulator::VAC_scan() {}

/////////

void Regulator::Pn_CVg_TransistorCalibration(double Vg_min , double Vg_max , double incr, int delay, string folder) {
	//const int param_num = 8;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &step_V};  ALL TO DOUBLE!!!!!!!!!!!!!!!!!!!!!!	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "Pn_CVg_TransistorCalibration.txt", 4, Vg_min, Vg_max, incr, (double)delay);
	//return;
	make_logs(folder, "Pn_CVg_TransistorCalibration started\nTime per point: " + to_string((float)ADC_BUF_SIZE_2 / ADC_TGT_FREQ) + " s");
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
	
	std::cout << endl << "Output directories created: " << folder + timestr << endl;
	std::cout << endl << point_num << " Data points expected:"  << endl;
	std::cout << endl << " Expexted time:" << (delay + ADC_BUF_SIZE_2 / 2) * point_num * 2 / 1000000 / 60 * 13 / 11 << "m" << endl;
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
	//const int param_num = 8;
	//double* tmp[param_num] = {&bias_, &freq, &crit_V, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &step_V};  ALL TO DOUBLE!!!!!!!!!!!!!!!!!!!!!!	
	//LoadParamsFromFile("../../Settings/", "TouchScan.txt", param_num, tmp);
	//SaveParamsToFile("../../Settings/", "R_CVg_TransistorCalibration.txt", 6, incr, Vg_min, Vg_max, Vsd_crit, Vbias_crit, (double)delay_us);
	//return;
	make_logs(folder, "R_CVg_TransistorCalibration started\nTime per point: " + to_string((float)ADC_BUF_SIZE_2 / ADC_TGT_FREQ) + " s");


	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 500, ADC_BUF_SIZE_2);
	std::cout << endl << " R_CVg_TransistorCalibration started..." << endl;
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	ZCard.SingleAnalogOut(0.0, Z_OUT_FINE);
	string timestr = get_time_string();
	double Vg = Vg_min, Vbias =  0, Vsd = 0;
	double noise;
	double offset = 0.004;// было 0.004
	std::filesystem::create_directories(folder + timestr);
	ofstream file;
	std::cout << endl << " Output directories created..." << endl;	
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
			
			ZCard.SingleAnalogOut(Vbias, Z_OUT_FINE);
			XYCard.StopReadStream();
			XYCard.StartReadStream();
			uwait(delay_us);
			
			//uwait(10*delay_us);// проверка влияния задержки в больших сопротивлениях - не помогло

			data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2000, ADC_BUF_SIZE_2);
			Vsd = data.Average(ADC_BUF_SIZE_2 / 2, R_CALIBR_CH);
			noise = data.Average(ADC_BUF_SIZE_2 / 2, NOISE_CH);

			file << Vsd << "   " << ZCard.cur_volt[0] << "   " << noise << "   " << Vg << endl;
			
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







