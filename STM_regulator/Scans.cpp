#include "Scans.h"

using namespace std;

VAC::VAC(int count) :points_num(count), FWvoltages(count, 0), BWvoltages(count, 0), BWcurrents(count, 0), FWcurrents(count, 0) {
	time_t rawtime;
	struct tm* timeinfo;
	char buffern[20];								// строка, в которой будет храниться текущая дата
	time(&rawtime);									// текущая дата в секундах
#pragma warning(suppress : 4996)
	timeinfo = localtime(&rawtime);					// текущее локальное время, представленное в структуре
	strftime(buffern, 20, "%x", timeinfo);
	string data = "../../scans/";
	data += buffern;
	data += "/";
	strftime(buffern, 20, "%H_%M", timeinfo);

	data += buffern;
	data += "/VAC";
	std::filesystem::create_directories(data);
	save_dir = data;
	fwname = "FW_" + to_string(points_num) + "_"
		+ ".dat";
	bwname = "BW_" + to_string(points_num) + "_"
		+ ".dat";
}; 
void VAC:: print_in_file(bool dir, ofstream& outstream) {
	if (dir)
		for (int i = 0; i < points_num; i++) {
			outstream << FWvoltages[i] << "	";
			outstream << FWcurrents[i] << "	";
			outstream << endl;
		}
	else
		for (int i = 0; i < points_num; i++) {
			outstream << BWvoltages[i] << "	";
			outstream << BWcurrents[i] << "	";
			outstream << endl;
		}
}
void VAC::set(int point, bool dir, ADC_Collect data, int ch_c , int ch_v ) {
	if (dir == FORWARD) {
		FWcurrents[point] = data.Average(8, ch_c);
		FWvoltages[point] = data.Average(8, ch_v);
	}
	else {
		BWcurrents[point] = data.Average(8, ch_c);
		BWvoltages[point] = data.Average(8, ch_v);
	}
}
double VAC::avrg(vector<double>& array, int start, int avrg_num ) {
	double tmp = 0;
	for (int i = 0; i < avrg_num; i++) {
		if (start + i < points_num)
			tmp += array[start + i];
	}
	tmp = tmp / avrg_num;
	return tmp;
}
void VAC::print_cout(int period ) {
	cout << "FWvoltages" << "	" << "FWcurrents" << "	" << "BWvoltages" << "	" << "BWcurrents" << endl;
	for (int i = 0; i < points_num / period; i++) {
		cout << avrg(FWvoltages, i * period, period) << "	" << avrg(FWcurrents, i * period, period) << "	" << avrg(BWvoltages, i * period, period) << "	" << avrg(BWcurrents, i * period, period) << endl;
	}

}
void VAC::print_(string name) {
	ofstream Forward, Backward;
	Forward.open(save_dir + "/VAC" + name + fwname, std::ofstream::out);
	Backward.open(save_dir + "/VAC" + name + bwname, std::ofstream::out);
	print_in_file(FORWARD, Forward);
	print_in_file(BACKWARD, Backward);

}


VANC::VANC(int count) : VAC(count) {};
void VANC::print_in_file(bool dir, ofstream& outstream) {
	if (dir)
		for (int i = 0; i < points_num; i++) {
			outstream << FWvoltages[i] << "	";
			outstream << FWcurrents[i] << "	";
			outstream << FWnoise[i] << "	";
			outstream << endl;
		}
	else
		for (int i = 0; i < points_num; i++) {
			outstream << BWvoltages[i] << "	";
			outstream << BWcurrents[i] << "	";
			outstream << BWnoise[i] << "	";
			outstream << endl;
		}
}
void VANC::set(int point, bool dir, ADC_Collect data, int ch_c , int ch_v , int ch_n ) {
	
	if (dir == FORWARD) {
		FWcurrents[point] = data.Average(8, ch_c);
		FWvoltages[point] = data.Average(8, ch_v);
		FWnoise[point] = data.Average(8, ch_n);
	}
	else {
		BWcurrents[point] = data.Average(8, ch_c);
		BWvoltages[point] = data.Average(8, ch_v);
		BWnoise[point] = data.Average(8, ch_n);
	}
}
void VANC::print_(string name) {//при выводе одной вахи в один файл
	ofstream Forward, Backward;
	Forward.open(save_dir + "/VANC" + name + fwname, std::ofstream::out);
	Backward.open(save_dir + "/VANC" + name + bwname, std::ofstream::out);
	print_in_file(FORWARD, Forward);
	print_in_file(BACKWARD, Backward);
}



Scan::Scan(double x_dim, double y_dim, double x_step, double y_step) :
	x_dim(x_dim),
	y_dim(y_dim),
	x_step(x_step),
	y_step(y_step),
	HFWplot(y_dim / y_step, vector<double>(x_dim / x_step, 0)),
	HBWplot(y_dim / y_step, vector<double>(x_dim / x_step, 0)),
	CFWplot(y_dim / y_step, vector<double>(x_dim / x_step, 0)),
	CBWplot(y_dim / y_step, vector<double>(x_dim / x_step, 0)),
	x_n(x_dim / x_step),
	y_n(y_dim / y_step)
{
	time_t rawtime;
	struct tm* timeinfo;
	char buffern[20];								// строка, в которой будет храниться текущая дата
	time(&rawtime);									// текущая дата в секундах
#pragma warning(suppress : 4996)
	timeinfo = localtime(&rawtime);					// текущее локальное время, представленное в структуре
	strftime(buffern, 20, "%x", timeinfo);
	string data = "../../scans/";
	data += buffern;
	data += "/";
	strftime(buffern, 20, "%H_%M", timeinfo);

	data += buffern;
	std::filesystem::create_directories(data);
	save_dir = data;
	fwname = "FW_XD" + to_string(x_dim) + "_XS" + to_string(x_step)
		+ "_YD" + to_string(y_dim) + "_YS" + to_string(y_step) + "_"
		+ buffern + ".dat";
	bwname = "BW_XD" + to_string(x_dim) + "_XS" + to_string(x_step)
		+ "_YD" + to_string(y_dim) + "_YS" + to_string(y_step) + "_"
		+ buffern + ".dat";
}
void Scan::SaveFiles(string save_d ) {

	if (save_d != "-") {
		save_dir = save_d;
	}


	ofstream Hforward, Hbackward, Cforward, Cbackward;
	Hforward.open(save_dir + "/H" + fwname, std::ofstream::out);
	Hbackward.open(save_dir + "/H" + bwname, std::ofstream::out);
	Cforward.open(save_dir + "/C" + fwname, std::ofstream::out);
	Cbackward.open(save_dir + "/C" + bwname, std::ofstream::out);
	for (int y = 0; y < y_n; y++) {

		for (int x = 0; x < x_n; x++) {

			Hforward << HFWplot[y][x] << "	";
			Hbackward << HBWplot[y][x] << "	";
			Cforward << CFWplot[y][x] << "	";
			Cbackward << CBWplot[y][x] << "	";
		}

		Hforward << endl;
		Hbackward << endl;
		Cforward << endl;
		Cbackward << endl;
	}

}
void Scan::SaveRow(int row, string save_d) {

	if (save_d != "-") {
		save_dir = save_d;
	}

	ofstream Hforward, Hbackward, Cforward, Cbackward;
	Hforward.open(save_dir + "/H" + fwname, std::ofstream::out | std::ios::app);
	Hbackward.open(save_dir + "/H" + bwname, std::ofstream::out | std::ios::app);
	Cforward.open(save_dir + "/C" + fwname, std::ofstream::out | std::ios::app);
	Cbackward.open(save_dir + "/C" + bwname, std::ofstream::out | std::ios::app);


	for (int x = 0; x < x_n; x++) {

		Hforward << HFWplot[row][x] << "	";
		Hbackward << HBWplot[row][x] << "	";
		Cforward << CFWplot[row][x] << "	";
		Cbackward << CBWplot[row][x] << "	";
	}

	Hforward << endl;
	Hbackward << endl;
	Cforward << endl;
	Cbackward << endl;
}


VAC_Scan::VAC_Scan(double x_dim, double y_dim, double x_step, double y_step, int VAC_count, int points_count) :
	Scan(x_dim, y_dim, x_step, y_step),
	FWVACs(y_n, vector< vector<VAC>>(x_n, vector<VAC>(VAC_count, points_count))),
	BWVACs(y_n, vector< vector<VAC>>(x_n, vector<VAC>(VAC_count, points_count))) {

}
void VAC_Scan::SaveFiles(string save_d ) {
	GwyfileObject* gwyf, * datafield, ** curves_gr, ** curves_sp, * graph, * selection, * spectr;
	GwyfileError* error = NULL;
	Scan::SaveFiles(save_d);
	ofstream VACstream;
	save_d += "/VACs";
	VACstream.open(save_dir, std::ofstream::out);

}