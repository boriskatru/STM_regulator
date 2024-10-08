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
	
	file.open(folder + VANC_SETTINGS, std::ios::in);
	file >> bias;
	for (int i = 0; i < 8; i++) {
		file >> target;
	}
	file >> P >> I >> D;
	//cout << endl << bias << "   " << target << "   " << P << "   " << I << "   " << D << "   "  << endl;
	r = CHTransform(target, bias);
	file.close();
}

void PID::save_settings(string folder){
	ofstream file;
	file.open(folder + PID_SETTINGS_FILE, std::ios::out);
	file << target << endl << bias << endl << P << endl << I << endl << D;
	file.close();
}

/////////

void Regulator::wait_clear_buf(Card& card, int cnt ) {
	for (int i = 0; i < cnt; i++)  card.SingleAnalogRead();
}

void Regulator::WriteProgressStatus(string name, double percent, double estimated_time)
{
	ofstream file;
	file.open(folder + PROGRESS_STATUS_FILE, std::ios::out);
	file << name << endl;
	file << percent << endl;
	file << estimated_time << endl;
	file << piezo.Position('x') << endl;
	file << piezo.Position('y') << endl;
	file << piezo.Position('z') << endl;
	file << ZCard.SingleAnalogRead()*10 << endl;
	file << biasDC << endl;
	file << biasAC << endl;
	file << frequency << endl;
	file.close();
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


Regulator::Regulator(double i_offset, double noise_limit_V, string folder):
	XYCard(1, ADC_CHANEL_CNT, ADC_BUF_SIZE_2),
	ZCard(),
	MFLI(),
	noise_limit_V(noise_limit_V),
	frequency(frequency), biasDC(0), biasAC(0),
	folder(folder) {
	SaveEquipmentStatus();
	ZCard.SingleAnalogOut(0, Z_OUT);
	ZCard.SingleAnalogOut(0, Z_OUT_FINE);

}
Regulator::~Regulator() {
	MHome();
	XYCard.FullStop();
}


/////////

void Regulator::MoveTo(double x_, double y_, double step_size, bool need_logs)
{
	if (need_logs) {
	WriteExecStatus(2);
	}
	piezo.MoveTo(Vecter(x_, y_, piezo.Position()), 100, step_size, ZCard, XYCard);
	if (need_logs) {
		string log = "";
		log += "X moved to: " + to_string(x_);
		log += "\n Y moved to: " + to_string(y_);
		make_logs(log);
		WriteExecStatus(0);
	}
}
void Regulator::MoveTo()
{
	WriteExecStatus(2);
	WriteProgressStatus("MOVING", 50, -1);
	const int param_num = 4;
	double x_, y_,  step_speed, delay;
	double* tmp[param_num] = { &x_, &y_, &step_speed, &delay };
	LoadParamsFromFile(MoveXY_SETTINGS, param_num, tmp);

	piezo.MoveTo(Vecter(x_, y_, piezo.Position()), delay, step_speed * DEFAULT_MICROSTEP_SIZE, ZCard, XYCard);
	
	string log = "";
	log += "X moved to: " + to_string(x_);
	log += "\n Y moved to: " + to_string(y_);
	make_logs(log);
	WriteExecStatus(0);
	WriteProgressStatus("MOVING", 100, 0);
}
void Regulator::MHome(double step_size, bool need_logs) {
	if (need_logs) {
		WriteExecStatus(2);
	}
	piezo.MoveTo(Vecter(0, 0, 0), 0, step_size, ZCard, XYCard);
	if (need_logs) {
		make_logs("Moved home(0,0,0)");
		WriteExecStatus(0);
		WriteProgressStatus("Homing", 100, 0);
	}

}
void Regulator::ZHome(double step_size) {
	piezo.MoveTo(Vecter(piezo.Position('X'), piezo.Position('Y'), 0), 0, step_size, ZCard, XYCard);
}
void Regulator::JHome() {
	piezo.JumpTo(Vecter(0, 0, 0), ZCard, XYCard);
}

int Regulator::GetStatus()
{
	ifstream file;
	int status;
	file.open(folder + EXEC_STATUS_FILE, std::ios::in);
	file >> status;
	file.close();
	return status;
}
bool Regulator::CheckStatus(string stop_message) {	
	while (GetStatus() == 1) {
		uwait(100000);
	}
	if (GetStatus() == 0) {
		MHome();
		make_logs(stop_message);
		return 1;
	}
	else return 0;
}

void Regulator::WriteExecStatus(int status)
{
	ofstream file;
	file.open(folder + EXEC_STATUS_FILE, std::ios::out);
	file << status << std::flush;
	file.close();
}
string Regulator::ReadSessionDirectory(string path)
{
	ifstream file;
	string tmp;
	file.open(folder + path, std::ios::in);
	file >> session_folder;
	file >> tmp;
	session_folder = session_folder + " " + tmp;
	file.close();
	return session_folder;
}
void Regulator::AddFileToSession(string path, string file)
{
	ReadSessionDirectory();
	cout <<"Filename " << file << " added to session path "<< session_folder + path <<endl;
	ofstream list;
	list.open(session_folder + path, std::ios_base::app);
	list << file << endl;
	list.close();
}
void Regulator::WriteVANCDirectory(string foldername, string path)
{
	ofstream file;
	file.open(folder + path, std::ios::out);
	file << foldername << std::flush;
	file.close();

}
void Regulator::ResetPIDFromFile()
{
	pid.reset_from_file(folder);
	if (biasAC != pid.bias) {
		biasAC = pid.bias;
		MFLI.setACAmplitude(biasAC);
	}	
	pid.set_zero_pos(pid.last_signal);
}
void Regulator::LoadParamsFromFile(string path , int count, double**arr)
{		
		ifstream file;
		file.open(folder + path, std::ios::in);	
		double tmp;		
		for (int i = 0; i < count; i++) {
			file >> tmp;				
			*arr[i] = tmp;
			//cout <<"Номер   "<< i << "		Указатель   " << arr[i] << "	Значение   " << *arr[i] << "	Значение   " << tmp << endl;
		}
		file.close();
}
void Regulator::SaveParamsToFile(string path, int count, ...)
{
	va_list vl;
	ofstream file;
	//cout << path << endl;
	file.open(folder + path, std::ios::out);
	
	va_start(vl, count);	
	for (int i = 0; i < count; i++) {	
		file << va_arg(vl, double) << endl;		
	}
	va_end(vl);

	file.close();
}
void Regulator::SaveEquipmentStatus(string path)
{
	ofstream file;
	file.open(folder + path, std::ios::out);
	file << ZCard.status << endl << XYCard.status << endl << MFLI.status;
	file.close();
}
void Regulator::ZStep(int dir, double step_size, bool makelogs) {
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
		make_logs("Z step " + to_string(dir ? step_size : -step_size)); 
	}
}
void Regulator::StepXY() {
	WriteExecStatus(2);
	const int param_num = 6;
	double x_steps, y_steps, step_size, crit_V, step_speed, delay;
	double* tmp[param_num] = { &x_steps, &y_steps, &step_size, &crit_V, &step_speed, &delay};
	LoadParamsFromFile(StepXY_SETTINGS, param_num, tmp);
	biasAC = 0.2;
	biasDC = 0.8;
	WriteProgressStatus("STEPS", 0, (x_steps + y_steps) / step_speed);
	MFLI.setACEnable();
	MFLI.setSignalEnable();
	MFLI.setDCAmplitude(0.8);
	MFLI.setACAmplitude(0.2);
	//SaveParamsToFile("../../Settings/", "StepXY.txt", 6, (double)x_steps, (double)y_steps, (double)need_logs, step_size, step_speed, delay);
	//return;

	for (int i = 0; i < abs(x_steps); i++) { 
		if (abs(ZCard.SingleAnalogRead()) < crit_V) {
			Step(X_, x_steps, step_size, step_speed);
			uwait(delay);
			if (CheckStatus("Steps stopped by user. "+ to_string(i)+" X steps done")) return;
			if (i % 20 == 0) WriteProgressStatus("STEPS", i * 100 / (x_steps + y_steps), (x_steps + y_steps - i) / step_speed);
		}
		else { 
			make_logs("AHTUNG!!!OBSTACLE!!!STEPS STOPPED!!!");
			return; 
		}
	}
	for (int i = 0; i < abs(y_steps); i++) {
		if (abs(ZCard.SingleAnalogRead()) < crit_V) {
			Step(Y_, y_steps, step_size, step_speed);
			uwait(delay);
			if (CheckStatus("Steps stopped by user. " + to_string(x_steps) + " X steps and" + to_string(i) + " Y steps done")) return;
			if (i % 20 == 0) WriteProgressStatus("STEPS", (i+ x_steps) * 100 / (x_steps + y_steps), (y_steps - i) / step_speed);
		}
		else {
			make_logs("AHTUNG!!!OBSTACLE!!!STEPS STOPPED!!!");
			return;
		}
	}
	
	string log = "";
	if (x_steps != 0) log += "X steps done: " + to_string(x_steps);
	if (y_steps != 0) log += "\n Y steps done: " + to_string(y_steps);
	make_logs(log);
	
	WriteExecStatus(0);
	WriteProgressStatus("STEPS", 100, 0);
}
void Regulator::StepXY(int x_steps, int y_steps, bool need_logs, double step_size, double step_speed, double delay, string folder) {
	WriteExecStatus(2);
	for (int i = 0; i < abs(x_steps); i++) {
		if (abs(ZCard.SingleAnalogRead()) < 0.5) {
			Step(X_, x_steps, step_size, step_speed);
			uwait(delay);
			if (CheckStatus("Steps stopped by user. " + to_string(i) + " X steps done")) return;
		}
		else {
			cout << "AHTUNG!!!  OBSTACLE!!! STEPS STOPPED!!!" << endl;
			make_logs("AHTUNG!!!OBSTACLE!!!STEPS STOPPED!!!");
			return;
		}
	}
	for (int i = 0; i < abs(y_steps); i++) {
		if (abs(ZCard.SingleAnalogRead()) < 0.5) {
			Step(Y_, y_steps, step_size, step_speed);
			uwait(delay);
			if (CheckStatus("Steps stopped by user. " + to_string(i) + " X steps done")) return;
		}
		else {
			cout << "AHTUNG!!!  OBSTACLE!!! STEPS STOPPED!!!" << endl;
			make_logs("AHTUNG!!!OBSTACLE!!!STEPS STOPPED!!!");
			return;
		}
	}
	if (need_logs) {
		string log = "";
		if (x_steps != 0) log += "X steps done: " + to_string(x_steps);
		if (y_steps != 0) log += "\n Y steps done: " + to_string(y_steps);
		make_logs(log);
		WriteExecStatus(0);
	}

}
/////////

void Regulator::Retract() {
	WriteExecStatus(2);
	const int param_num = 8;
	double x_steps, y_steps, step_size, crit_V, step_speed, delay, steps, step_incr;
	double* tmp[param_num] = { &x_steps, &y_steps, &step_size, &crit_V, &step_speed, &delay, &steps, &step_incr};
	LoadParamsFromFile(Retract_SETTINGS, param_num, tmp);
	WriteProgressStatus("RETRACT", 0, steps);
	MFLI.setACEnable();
	MFLI.setSignalEnable();
	MFLI.setDemodFilterFreq(20000);
	MFLI.setOscFreq(5000);
	MFLI.setDCAmplitude(0.8);
	MFLI.setACAmplitude(0.2);
	
	int step = -2;
	ZHome();
	double height = rise(0.2, crit_V, DEFAULT_MICROSTEP_SIZE*4);
	ZHome();
	double st_sz = max(height / 1.25 - 0.2, 0.4);
	if (CheckStatus("Steps stopped by user. " + to_string(step) + " X steps done")) return;
	for (double sz = st_sz; sz <= 5; sz += step_incr) {

		ZCard.SingleAnalogOut(min(5, sz * 1.7));
		ZCard.SingleAnalogOut(sz - 0.25);
		ZStep(-1, sz, false);
		step += 1;
		WriteProgressStatus("RETRACT", step * 100/steps,  steps - step);
		if (CheckStatus("Steps stopped by user. " + to_string(step) + " X steps done")) return;

	}
	while (step < steps) {
		ZStep(-1, 5, false);
		step++;
		WriteProgressStatus("RETRACT", step * 100 / steps, steps - step);
		if (CheckStatus("Steps stopped by user. " + to_string(step) + " X steps done")) return;
	}
	make_logs("Rectracted steps: "+ to_string(steps));
	
	WriteProgressStatus("RETRACT",  100 , 0);
	WriteExecStatus(0);
}
double Regulator::rise( double bwa, double target_V, double djump) {
	

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
			cout << "No touch signal" << endl;
			ZHome(MIN_STEP_SIZE * 2);
			break;
		}
	}

	
	piezo.Move(Vecter(0, 0, -bwa), 0, 5 * MIN_STEP_SIZE, ZCard, XYCard);
	ZHome();
	ZCard.StopReadStream();
	make_logs(" Check rise | Touch height: " + to_string(last_height));
	return last_height;
}
double Regulator::Landing() {
	WriteExecStatus(2);	
	const int param_num = 9;
	double target_V, step_size, step_speed, delay;
	double* tmp[param_num] = { &biasAC, &biasDC, &frequency, &sinc_on, &filter_freq, &step_size, &target_V, &step_speed, &delay };
	LoadParamsFromFile(Landing_SETTINGS, param_num, tmp);
	WriteProgressStatus("LANDING", 50, -1);
	target_V = target_V / 10;
	if (sinc_on == 1)	MFLI.setSincEnable();
	else MFLI.setSincDisable();
	MFLI.setACEnable();
	MFLI.setSignalEnable();
	MFLI.setDemodFilterFreq(filter_freq);
	MFLI.setOscFreq(frequency);
	MFLI.setDCAmplitude(biasDC);
	MFLI.setACAmplitude(biasAC);
	
	make_logs("Landing started");

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
		if (piezo.Position('Z') >= step_size) {
			//uwait(delay_micro);
			stp_count++;
			piezo.ZJumpTo(0, ZCard);
			if (CheckStatus("Landing stopped by user")) return 0;
			
			for (int i = 0; i < 10; i++) {
				uwait(delay/10);
				ZCard.SingleAnalogRead();
			}


		}
	}

	
	piezo.Move(Vecter(0, 0, -0.2), 0, 8 * djump, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	piezo.JumpTo(Vecter(0, 0, 0), ZCard, XYCard);
	std::cout << "Touch current signal: " << ZCard.data_[0] << endl;
	std::cout << "Touch_height: " << last_height << endl;
	std::cout << "Landing done" << endl;
	std::cout << "Steps:" << stp_count << endl;
	//getchar(); getchar();
	ZCard.StopReadStream();
	make_logs("Landing completed\nSteps done:"+ to_string(stp_count)+ "\n Touch height: "+ to_string(last_height));
	WriteExecStatus(0);
	WriteProgressStatus("LANDING", 100, 0);
	return last_height;
}


/////////

void Regulator::IntPID(double bias_, double target_V, double duration_us, double pid_log_offset, double start_offset) {

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
double Regulator::IntPID_exp(double bias_, double target_V, double duration_us, double start_pos, int polarity,  double touch_lim, int update_delay) {
	pid.reset_from_file();
	biasAC = bias_;
	int i = 0;
	Timer tmr;
	if (duration_us == -1) {
		pid.set_zero_pos(start_pos);
		while (true) {
			piezo.ZFJumpTo(pid.signal(CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, biasAC), tmr.get_loop_interval()), ZCard);
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
							make_logs("VANC measurements stopped by user ");
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
			piezo.ZFJumpTo(pid.signal(CHTransform(LimCatch(polarity * ZCard.SingleAnalogRead(), touch_lim) * I_to_nA, biasAC), tmr.get_loop_interval()), ZCard);
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
							make_logs("VANC measurements stopped by user ");
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
	//make_logs("Experimental PID stopped ");
	return pid.last_signal;

}


/////////

void Regulator::VANC_PID()
{
	WriteExecStatus(2);
	const int param_num = 9;
	double count, target_V, buffsize, pre_wait;
	double* tmp[param_num] = { &biasAC, &biasDC, &frequency, &sinc_on, &filter_freq, &count, &buffsize, &pre_wait, &target_V};
	LoadParamsFromFile(VANC_SETTINGS, param_num, tmp);
	//cout << endl << biasAC << "   " << biasDC << "   " << frequency << "   " << sinc_on << "   " << filter_freq << "   " << count << "   " << buffsize << "   " << pre_wait << "   " << target_V << endl;
	WriteProgressStatus("VANC measurments", 0, pre_wait+ (ADC_BUF_SIZE_2 / ADC_TGT_FREQ+1) * count);
	target_V = target_V / 10;
	if (sinc_on == 1)	MFLI.setSincEnable();
	else MFLI.setSincDisable();
	MFLI.setACEnable();
	MFLI.setSignalEnable();
	MFLI.setDemodFilterFreq(filter_freq);
	MFLI.setOscFreq(frequency);
	MFLI.setDCAmplitude(biasDC);
	MFLI.setACAmplitude(biasAC);
	double delay = 1 + ADC_BUF_SIZE_2 / ADC_TGT_FREQ;

	ADC_Collect data = XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
	Timer tmr;
	string timestr = get_time_string();
	make_logs("VANC measurements with PID started \nParameters: \n	BiasAC: " + to_string(biasAC) + "\n	Frequency: " + to_string(frequency)
		+ "\n	Target_V: " + to_string(target_V) + "\n	Count: " + to_string(count) + "\n	Delay: " + to_string(delay)
		+ "\n	X: " + to_string(piezo.Position('X')) + "	Y: " + to_string(piezo.Position('Y')));
	pid.reset(target_V, biasAC);
	//pid.save_settings();
	WriteVANCDirectory(folder + SCAN_FOLDER + timestr);
	double  height = IntPID_exp(biasAC, target_V, pre_wait * 1000000, 0);
	//cout << "done   " << 0 << " of " << count << " VANCS" << endl;
	for (int i = 1; i <= count; i++) {
		tmr.get_loop_interval();
		XYCard.StartReadStream();
		height = IntPID_exp(biasAC, target_V, delay * 1000000, height);
		data = XYCard.AnalogRead(0, ADC_BUF_SIZE_2);
		XYCard.StopReadStream();
		cout << "done   " << i << " of " << count << " VANCS" << endl;
		data.print_f_VANC("VANC_" + to_string(i),".bin", folder + SCAN_FOLDER + timestr);

		//cin >> stop;
		//if (stop) break;
		cout << "loop time : " << tmr.get_loop_interval() / 1000000 << endl;
		WriteProgressStatus("VANC measurments", 100*(pre_wait + i*(delay + 0.2) )/(pre_wait + (delay+0.2) * count), (delay + 0.2) * (count-i));
		if(GetStatus() < 2) { 
			piezo.Move(Vecter(0, 0, -0.15), 100, MIN_STEP_SIZE / 2, ZCard, XYCard);
			height = height - 0.15;
			while (GetStatus() < 2) {
				if (GetStatus() == 0) {
					ZHome();					
					make_logs("VANC measurements stopped by user ");
					return;
				}
				uwait(100000);
			}
		}

	}
	ZHome();	
	make_logs("VANC measurements done ");
	WriteExecStatus(0);
	WriteProgressStatus("VANC", 100, 0);
}
/////////

void Regulator::CapStepScan() {
	WriteExecStatus(2);
	const int param_num = 12;
	double  step_V, crit_V, x_steps, X_step_sz, y_steps, Y_step_sz, fw_bw;
	double* tmp[param_num] = { &biasAC, &biasDC, &frequency, &sinc_on, &filter_freq, &x_steps, &y_steps, &X_step_sz, &Y_step_sz, &crit_V, &step_V ,&fw_bw };
	LoadParamsFromFile(CapStepScan_SETTINGS, param_num, tmp);
	WriteProgressStatus("CAPACITANCE SCAN", 0, 100);
	crit_V = crit_V / 10;
	if (sinc_on == 1)	MFLI.setSincEnable();
	else MFLI.setSincDisable();
	MFLI.setACEnable();
	MFLI.setSignalEnable();
	MFLI.setDemodFilterFreq(filter_freq);
	MFLI.setOscFreq(frequency);
	MFLI.setDCAmplitude(biasDC);
	MFLI.setACAmplitude(biasAC);
	string log = "Capasitance step scan started \nParameters: \n	Bias: " + to_string(biasAC) +
		"\n	X Step size: " + to_string(X_step_sz) +
		"\n	Y Step size: " + to_string(Y_step_sz) +
		"\n	X size: " + to_string(x_steps) +
		"\n	Y size:" + to_string(y_steps);
	make_logs(log);
	
	
	Scan scan(x_steps, y_steps, 1,1, biasAC);
	AddFileToSession(CAP_SCAN_LIST_NAME, scan.save_dir + "H" + scan.fwname);
	//Scan scan_bw(x_steps*X_FW_BW, y_steps, 1, 1);
	/*Обнуляем всё*/
	
	cout << "signal : " << ZCard.SingleAnalogRead() << endl;
	
	int fwx_steps = 0;
	int bwx_steps = 0;
	/*Начало сканирования*/
	for (int y = 0; y < scan.y_n; y++) {
		/*передний ход:*/

		for (int x = 0; x < scan.x_n; x++) {

			uwait(50000);	
			scan.HFWplot[y][x] = piezo.Position();
			scan.CFWplot[y][x] = ZCard.SingleAnalogRead();
			if (scan.CFWplot[y][x] > crit_V) {
				StepXY(-X_step_sz, -Y_step_sz, false, step_V);				
				make_logs("!!!WARNING!!!  Obstacle detected! Capasitance step scan stopped" );
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
		std::cout << "current y:	" << y << "	of	" << scan.y_n << endl;
		scan.SaveRow(y);
		WriteProgressStatus("CAPACITANCE SCAN", y * 100 / scan.y_n, 100);
		//scan_bw.SaveRow(y);
	}
	
	StepXY(0, -Y_step_sz * ((Y_step_sz >= 0) ? Y_FW_BW : 1 / Y_FW_BW) * y_steps, false, step_V); // если закоментированно, то не возвращается по Y!!
	ZCard.StopReadStream();
	//scan.SaveFiles();	
	make_logs("Capasitance step scan done ");
	MHome();
	WriteExecStatus(0);
	WriteProgressStatus("CAPACITANCE SCAN",100, 0);
	//getchar(); getchar();
}

void Regulator::TouchScan() {
	WriteExecStatus(2);
	const int param_num = 17;
	double h_lim = 4.9;
	double  bwa, crit_V, x_start, x_step, x_stop, y_start, y_step, y_stop, step_speed, pre_wait, h_diff_lim, up_mult;
	double* tmp[param_num] = {&biasAC, &biasDC, &frequency, &sinc_on, &filter_freq, &bwa, &crit_V, &x_start, &x_step, &x_stop, &y_start, &y_step, &y_stop, &step_speed, &pre_wait , &h_diff_lim, &up_mult };
	LoadParamsFromFile(TouchScan_SETTINGS, param_num, tmp);

	crit_V = crit_V / 10;
	if (sinc_on == 1)	MFLI.setSincEnable();
	else MFLI.setSincDisable();
	MFLI.setACEnable();
	MFLI.setSignalEnable();
	MFLI.setDemodFilterFreq(filter_freq);
	MFLI.setOscFreq(frequency);
	MFLI.setDCAmplitude(biasDC);
	MFLI.setACAmplitude(biasAC);

	string log = "Touch scan started \nParameters: \n	BiasDC: " + to_string(biasDC) +
		"\n	BWA: " + to_string(bwa) + "\n	Target_V: " + to_string(crit_V) +
		"\n	Step speed: " + to_string(step_speed) +
		"\n	X: " + to_string(x_start) + " : " + to_string(x_step) + " : " + to_string(x_stop) +
		"\n	Y: " + to_string(y_start) + " : " + to_string(y_step) + " : " + to_string(y_stop);
	make_logs(log);
		
	Scan scan(abs(x_stop - x_start), abs(y_stop - y_start), abs(x_step), abs(y_step), biasDC);
	WriteProgressStatus("TOUCH SCAN", 0, 0.3 / step_speed * scan.y_n * scan.x_n * bwa + pre_wait);
	AddFileToSession(SCAN_LIST_NAME, scan.save_dir + "H" +  scan.fwname);
	/*Идём на старт*/
	bool is_touch = false;
	piezo.MoveTo(Vecter(x_start, y_start, 0), 0, DEFAULT_MICROSTEP_SIZE, ZCard, XYCard);
	/*Касание и выдержка*/

	while (!is_touch) {
		piezo.ZJump(DEFAULT_MICROSTEP_SIZE, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);
		if (piezo.Position() < 2 * bwa) is_touch = 0;		
		if (piezo.Position() >= h_lim) {
			cout << "No touch signal" << endl;
			return;
		}
		if (CheckStatus("Touch scan stopped by user")) return;
	}
	
	while (is_touch) {
		piezo.ZJump(-4 * MIN_STEP_SIZE, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);
		if (piezo.Position() < 2 * bwa) is_touch = 0;
		ZCard.SingleAnalogRead();
	}

	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, MIN_STEP_SIZE, ZCard, XYCard);

	
	for (int i = 0; i < pre_wait; i++) {
		Sleep(1000); // ждём pre_wait секунд
		if (CheckStatus("Touch scan stopped by user")) return;
	}

	is_touch = (ZCard.SingleAnalogRead() > crit_V);
	while (is_touch) {

		piezo.ZJump(-4 * MIN_STEP_SIZE, ZCard);
		is_touch = (ZCard.SingleAnalogRead() > crit_V);
		if (piezo.Position() < bwa) is_touch = 0;
		ZCard.SingleAnalogRead();
	}

	is_touch = (ZCard.SingleAnalogRead() > crit_V);
	cout << "signal : " << ZCard.SingleAnalogRead() << "		Crit_V = " << crit_V << endl;
	cout << "is touch : " << is_touch << endl;
	/*Начало сканирования*/
	for (int y = 0; y < scan.y_n; y++) {
		/*передний ход:*/

		for (int x = 0; x < scan.x_n; x++) {

			uwait(50);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {
				piezo.ZJump(MIN_STEP_SIZE * step_speed, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);				
				if (piezo.Position() > min((h_lim + X_PLANE_TG * x_step * x + Y_PLANE_TG * y_step * y),4.999)) is_touch = 1;
			}

			scan.HFWplot[y][x] = piezo.Position();			
			scan.CFWplot[y][x] = ZCard.data_[0] * 1000;

			//uwait(100);

			while (is_touch) {

				piezo.ZJump(-4 * MIN_STEP_SIZE * step_speed, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);
			
				if (piezo.Position() < bwa) is_touch = 0;
				ZCard.SingleAnalogRead();
			}
			piezo.Move(Vecter(0, 0, -bwa), 0, MIN_STEP_SIZE * step_speed* 4, ZCard, XYCard);
			piezo.Move(Vecter(x_step, 0,  0), 0, 2 * MIN_STEP_SIZE * step_speed, ZCard, XYCard);
			if (x == 0 && y == 0) h_lim = scan.HFWplot[y][x]+ h_diff_lim;
		}
		/*задний ход:*/
		for (int x = scan.x_n - 1; x >= 0; x--) {
			
			uwait(50);
			is_touch = (ZCard.SingleAnalogRead() > crit_V);

			
			if (piezo.Position() < bwa) is_touch = 0;
			while (!is_touch) {

				piezo.ZJump(MIN_STEP_SIZE * step_speed, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);				
				if (piezo.Position() > min((h_lim + X_PLANE_TG * x_step * x + Y_PLANE_TG * y_step * y), 4.999)) is_touch = 1;
			}

			scan.HBWplot[y][x] = piezo.Position();
			scan.CBWplot[y][x] = ZCard.data_[0] * 1000;

			//uwait(100);

			while (is_touch) {

				piezo.ZJump(-4 * MIN_STEP_SIZE * step_speed, ZCard);
				is_touch = (ZCard.SingleAnalogRead() > crit_V);

				if (piezo.Position() < bwa) is_touch = 0;
				ZCard.SingleAnalogRead();
			}
			piezo.Move(Vecter(0, 0, -bwa), 0, 4 * MIN_STEP_SIZE * step_speed, ZCard, XYCard);
			piezo.Move(Vecter(-x_step, 0, 0), 0, 2 * MIN_STEP_SIZE * step_speed, ZCard, XYCard);
		}

		piezo.Move(Vecter(0, y_step, 0), 0, MIN_STEP_SIZE* step_speed, ZCard, XYCard);
		std::cout << "current y:	" << y << "	of	" << scan.y_n << endl;
		scan.SaveRow(y);
		WriteProgressStatus("TOUCH SCAN", y * 100 / scan.y_n, 0.3 / step_speed * (scan.y_n - y) * scan.x_n * bwa );
		if (GetStatus() < 2) {
			piezo.Move(Vecter(0, 0, -up_mult * bwa), 100, MIN_STEP_SIZE * step_speed , ZCard, XYCard);
			if (CheckStatus("Touch scan stopped by user")) return;
		}
	}

	piezo.Move(Vecter(0, 0, -up_mult * bwa), 0, MIN_STEP_SIZE * step_speed / 4, ZCard, XYCard);
	MHome();
	ZCard.StopReadStream();
	//scan.SaveFiles();
	make_logs("Touch scan done ");
	WriteExecStatus(0);
	WriteProgressStatus("STEPS", 100, 0);
}


/////////

void Regulator::Pn_CVg_TransistorCalibration() {
	WriteExecStatus(2);
	
	const int param_num = 7;
	double  Vg_min, Vg_max, incr, delay, nothing;
	double* tmp[param_num] = {&Vg_min, &Vg_max, &incr, &nothing, &nothing, &nothing, &delay};
	LoadParamsFromFile(PNCalibr_SETTINGS, param_num, tmp);
	WriteProgressStatus("CALIBRATION NOISE POWER FROM GATE VOLTAGE", 0, 100);
	make_logs("Pn_CVg_TransistorCalibration started\nTime per point: " + to_string((float)ADC_BUF_SIZE_2 / ADC_TGT_FREQ) + " s");
	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2000, ADC_BUF_SIZE_2);
	int point_num = (Vg_max - Vg_min) / incr;
	double dispersion = 0;
	double max_delta = 0;
	
	vector<double> noise(point_num, point_num);
	vector<double> volts(point_num, point_num);
	string timestr = get_time_string();
	std::filesystem::create_directories(MAIN_FOLDER + "scans/" + timestr);
	ofstream file, file_Back;
	
	std::cout << endl << "Output directories created: " << MAIN_FOLDER + "scans/" + timestr << endl;
	std::cout << endl << point_num << " Data points expected:"  << endl;
	std::cout << endl << " Expexted time:" << (delay + ADC_BUF_SIZE_2 / 2) * point_num * 2 / 1000000 / 60 * 13 / 11 << "m" << endl;
	ZCard.SingleAnalogOut(0.0, Z_OUT_FINE);
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	uwait(1000000);
	file.open(MAIN_FOLDER + "scans/" + timestr + "/" + "Noise_D.dat", std::ofstream::out);
	file_Back.open(MAIN_FOLDER + "scans/" + timestr + "/" + "Noise_B.dat", std::ofstream::out);
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
		WriteProgressStatus("CALIBRATION NOISE POWER FROM GATE VOLTAGE", i * 100 / 2 / point_num, ADC_BUF_SIZE_2 / ADC_TGT_FREQ * (2 *point_num - i));
		if (GetStatus() < 2) {			
			if (CheckStatus("Touch scan stopped by user")) return;
		}
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
		WriteProgressStatus("CALIBRATION NOISE POWER FROM GATE VOLTAGE", i * 100 / point_num + 50, ADC_BUF_SIZE_2 / ADC_TGT_FREQ * (point_num - i));
		if (GetStatus() < 2) {
			if (CheckStatus("Touch scan stopped by user")) return;
		}
	}
	std::cout << "data printed in file Noise_B.dat " << endl;
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	make_logs("Pn_CVg_TransistorCalibration completed");
	WriteExecStatus(0);
	WriteProgressStatus("CALIBRATION NOISE POWER FROM GATE VOLTAGE", 100, 0);
}
void Regulator::R_CVg_TransistorCalibration() {
	WriteExecStatus(2);
	const int param_num = 7;
	double  Vg_min, Vg_max, incr, delay_us, Vsd_crit, Vbias_crit, nothing;
	double* tmp[param_num] = { &Vg_min, &Vg_max, &incr, &Vsd_crit, &Vbias_crit, &nothing, &delay_us };
	LoadParamsFromFile(RCalibr_SETTINGS, param_num, tmp);
	WriteProgressStatus("CALIBRATION TRANSISTOR RESISTANCE FROM GATE VOLTAGE", 0, 100);
	make_logs("R_CVg_TransistorCalibration started\nTime per point: " + to_string((float)ADC_BUF_SIZE_3 / ADC_TGT_FREQ) + " s");


	ADC_Collect data = XYCard.AnalogRead(ADC_BUF_SIZE_3 / 500, ADC_BUF_SIZE_3);
	std::cout << endl << " R_CVg_TransistorCalibration started..." << endl;
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	ZCard.SingleAnalogOut(0.0, Z_OUT_FINE);
	string timestr = get_time_string();
	double Vg = Vg_min, Vbias =  0, Vsd = 0;
	double noise;
	double offset = 0.008;// было 0.004
	std::filesystem::create_directories(MAIN_FOLDER + "scans/" + timestr);
	ofstream file;
	std::cout << endl << " Output directories created..." << endl;	
	XYCard.StopReadStream();
	XYCard.StartReadStream();
	int count = 0, vac_num = (Vg_max - Vg_min) / incr;
	int dir = -1;
	for (double Vg = Vg_min; Vg < Vg_max; Vg += incr) {
		count++;
		file.open(MAIN_FOLDER + "scans/" + timestr + "/" + "VAC_Vg_" + to_string(Vg) + ".dat", std::ofstream::out);
		Vbias = 0;
		ZCard.SingleAnalogOut(Vg, Z_OUT);
		ZCard.SingleAnalogOut(Vbias, Z_OUT_FINE);

		dir = -1;
		uwait(delay_us);
		while ((dir < 1) || (Vbias <= MIN_STEP_SIZE)) {	
			if (GetStatus() < 2) {
				if (CheckStatus("Touch scan stopped by user")) return;
			}
			ZCard.SingleAnalogOut(Vbias, Z_OUT_FINE);
			XYCard.StopReadStream();
			XYCard.StartReadStream();
			uwait(delay_us);
			
			//uwait(10*delay_us);// проверка влияния задержки в больших сопротивлениях - не помогло

			data = XYCard.AnalogRead(ADC_BUF_SIZE_3 / 2000, ADC_BUF_SIZE_3);
			Vsd = data.Average(ADC_BUF_SIZE_3 / data.ch_count, BIAS_CH);  //БЫЛ R_CALIBR_CH!!!! ВЕРНУТЬ СРОЧНО!!!!!!
			noise = data.Average(ADC_BUF_SIZE_3 / data.ch_count, NOISE_CH);

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
		std::cout << count << "  of  " << vac_num << " VACs done" << endl;
		file.close();
		WriteProgressStatus("CALIBRATION TRANSISTOR RESISTANCE FROM GATE VOLTAGE", count/ vac_num, 100);
	}
	ZCard.SingleAnalogOut(Vg_min, Z_OUT);
	ZCard.SingleAnalogOut(0, Z_OUT_FINE);
	make_logs("R_CVg_TransistorCalibration completed");
	WriteExecStatus(0);
	WriteProgressStatus("CALIBRATION TRANSISTOR RESISTANCE FROM GATE VOLTAGE", 100, 10);
}







