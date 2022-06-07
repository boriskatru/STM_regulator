#include <fstream>
#include "LCard.h"

using namespace std;

ADC_Collect::ADC_Collect(int ch_count, int ADC_BUF_SIZE ) :
	average(ch_count, 0),
	ch_count(ch_count),
	s_ch_bufsz(ADC_BUF_SIZE / ch_count),
	err_cnt(0),
	input(ch_count, vector<double>(ADC_BUF_SIZE / ch_count, 0))/*, swap(0)*/ {
	current_data = (double*)malloc(sizeof(double) * ADC_BUF_SIZE);
	for (int i = 0; i < ADC_BUF_SIZE; i++) {
		current_data[i] = 0;
	}
}
void ADC_Collect::parse_channels() {
	bool is_same = 1;
	for (int i = 0; i < ch_count; i++) {
		for (int k = 0; k < s_ch_bufsz; k++) {
			if (input[i][k] != current_data[ch_count * k + i]) is_same = 0;
			input[i][k] = current_data[ch_count * k + i];
		}
	}
	if (is_same)
		err_cnt++;
	else err_cnt = 0;
	if (err_cnt > S_CNT_CRIT_NUM) {
		cerr << "Ошибка платы ввыода-вывода. Нет сигнала. Перезагрузите компьютер." << endl;
		cerr << "Same buffer for " << err_cnt << "times";
		getchar(); getchar();
	}
}
double ADC_Collect::Average(int count , int ch ) {
	average[ch] = 0;
	for (int i = s_ch_bufsz - 1; i > max(s_ch_bufsz - 1 - count, 0); i--) {
		average[ch] += input[ch][i];
	}

	average[ch] = average[ch] / min(count, s_ch_bufsz);
	return average[ch];
}
void ADC_Collect::show() {

	cout << endl << endl << endl;
	for (int i = 0; i < ch_count; i++) {
		cout << endl << "CHANNEL " << i << ":" << endl;
		for (int k = 0; k < s_ch_bufsz; k++) {

			cout << input[i][k] << endl;
		}
	}
}


void ADC_Collect::print_f(string filename, string directory)
{
	std::filesystem::create_directories(directory);
	ofstream file;
	file.open(directory + "/" + filename, std::ofstream::out);
	/*for (int i = 0; i < ch_count; i++) {
		cout << "CHANNEL " << i << ":" << "	";
	}*/
	
	for (int k = 0; k < s_ch_bufsz; k++) {
		file << endl;
		for (int i = 0; i < ch_count; i++) {
			file << input[i][k] << "	";
		}
	}
	
}

LCard::LCard(int card_No , int ADC_CH_COUNT, int ADC_BUF_SIZE) : ADC_BUF_SIZE(ADC_BUF_SIZE), data(ADC_CH_COUNT, ADC_BUF_SIZE), cur_volt(2, 0), next_lch(0){
	buf = (uint32_t*)calloc(ADC_BUF_SIZE, sizeof(uint32_t));
	get_list_res = L502_GetSerialList(serial_list, MAX_MODULES_CNT, L502_GETDEVS_FLAGS_ONLY_NOT_OPENED, NULL);
	if (get_list_res < 0)
	{
		cerr << "Error " << get_list_res << "    Ошибка получения списка серийных номеров " << endl;
	}
	else if (get_list_res == 0)
	{
		cout << "Не найдено ни одного модуля LCard" << endl;
	}
	else
	{
		cout << "Найдено " << get_list_res << " модулей LCard " << endl;
		cout << "serial_1= " << serial_list[0] << endl << "serial_2= " << serial_list[1] << endl;
		/*getchar();getchar();*/

	}
	hnd = L502_Create();

	serial = serial_list[card_No - 1];

	cout << "Card" << card_No << "  have serial  " << serial << endl;
	err = L502_Close(hnd);
	if (err != 0) cerr << "Ошибка  " << err << "  в L502_Close()" << endl;
	err = L502_Open(hnd, serial);
	if (err != 0) cerr << "Ошибка  " << err << "  в L502_Open()" << endl;

	err = L502_SetLChannelCount(hnd, ADC_CH_COUNT);
	for (int i = 0; i < ADC_CH_COUNT; i++) {
		if (!err)
		{
			/* первый логический канал соответствует измерению 1 канала
			относительно общей земли */
			err = L502_SetLChannel(hnd, i, i, L502_LCH_MODE_COMM, L502_ADC_RANGE_10, 0);
			if (err)
				cout << "Ошибка в L502_SetLChannel()" << err << endl;
		}
	}

	err = L502_AsyncOutDac(hnd, L502_DAC_CH1, 0.0, 0x0001) + L502_AsyncOutDac(hnd, L502_DAC_CH2, 0.0, 0x0001);
	while (err != 0) {
		cerr << "Ошибка  " << err / 2 << "  в L502_AsyncOutDac()" << endl;
		err = L502_Close(hnd);
		if (err != 0) cerr << "Ошибка  " << err << "  в L502_Close()" << endl;
		err = L502_Open(hnd, serial);
		if (err != 0) cerr << "Ошибка  " << err << "  в L502_Open()" << endl;
		err = L502_AsyncOutDac(hnd, L502_DAC_CH1, 0.0, 0x0001) + L502_AsyncOutDac(hnd, L502_DAC_CH2, 0.0, 0x0001);
	}
	err = L502_SetAdcFreq(hnd, &ADC_COLLECT_FREQ, &ADC_FRAME_FREQ);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_SetAdcFreq()" << endl;
	else cout << "ADC_COLLECT_FREQ = " << ADC_COLLECT_FREQ << endl << "ADC_FRAME_FREQ = " << ADC_FRAME_FREQ << endl;

	err = L502_StreamsDisable(hnd, L502_STREAM_DIN);
	err = L502_StreamsDisable(hnd, L502_STREAM_DOUT);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_StreamsDisable()" << endl;
	err = L502_StreamsEnable(hnd, L502_STREAM_ADC);
	err = L502_StreamsEnable(hnd, L502_STREAM_DAC1);
	err = L502_StreamsEnable(hnd, L502_STREAM_DAC2);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_StreamsEnable()" << endl;
	L502_SetDmaIrqStep(hnd, L502_DMA_CH_IN, 9000000);
	L502_SetDmaIrqStep(hnd, L502_DMA_CH_OUT, 9000000);
	L502_SetDmaBufSize(hnd, L502_DMA_CH_IN, ADC_BUF_SIZE);
	L502_SetDmaBufSize(hnd, L502_DMA_CH_OUT, 16);
	L502_Configure(hnd, 0);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_Configure()" << endl;
	
	
}
/// <summary>
/// Установка настроек платы (см документацию к плате, необходимо отправлять нужный флаг)
/// </summary>
/// <param name="flags"> флаг настройки </param>
/// 
void LCard::SetMode(uint32_t flags) {
	err = L502_Configure(hnd, flags);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_Configure(" << flags << ")" << endl;
}
/// <summary>
/// Единичный вывод на канал DAC
/// </summary>
/// <param name="data"> Задаваемый сигнал напряжения</param>
/// <param name="channel"> Номер канала DAC </param>
/// <param name="flags"></param>
void LCard::SingleAnalogOut(double data, uint32_t channel , uint32_t flags ) {
	err = L502_AsyncOutDac(hnd, channel, data, flags);
	cur_volt[channel] = data;
	while (err != 0) {
		cerr << "Ошибка  " << err << "  в L502_AsyncOutDac()" << endl;
		err = L502_Close(hnd);
		if (err != 0) cerr << "Ошибка  " << err << "  в L502_Close()" << endl;
		err = L502_Open(hnd, serial);
		if (err != 0) cerr << "Ошибка  " << err << "  в L502_Open()" << endl;
		err = L502_AsyncOutDac(hnd, channel, data, 0x0001);
	}
}
void LCard::SingleDigitalOut(uint32_t val, uint32_t mask) {
	err = L502_AsyncOutDig(hnd, val, mask);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_AsyncOutDig(" << mask << ")" << endl;
}
double LCard::AsyncSingleAnalogRead(int channel, double freq , uint32_t tout , uint32_t flags ) {
	if ((freq != -1) && (freq != ADC_COLLECT_FREQ)) {
		ADC_COLLECT_FREQ = freq;
		ADC_FRAME_FREQ = freq / 2;
		err = L502_SetAdcFreq(hnd, &ADC_COLLECT_FREQ,
			&ADC_FRAME_FREQ);
		if (err != 0) cerr << "Ошибка  " << err << " в L502_SetAdcFreq(" << freq << ")" << endl;
		else cout << "ADC_COLLECT_FREQ = " << ADC_COLLECT_FREQ << endl << "ADC_FRAME_FREQ = " << ADC_FRAME_FREQ << endl;
		//getchar(); getchar();
	}
	err = L502_AsyncGetAdcFrame(hnd, flags, tout, data.current_data);
	//if (err != 0) cerr << "Ошибка  " << err << " в L502_AsyncGetAdcFrame(" << flags << ")" << endl;
	return data.current_data[channel];
}
double* LCard::AsyncAnalogRead(double freq , uint32_t tout , uint32_t flags ) {
	if ((freq != -1) && (freq != ADC_COLLECT_FREQ)) {
		ADC_COLLECT_FREQ = freq;
		ADC_FRAME_FREQ = freq / 2;
		err = L502_SetAdcFreq(hnd, &ADC_COLLECT_FREQ,
			&ADC_FRAME_FREQ);
		if (err != 0) cerr << "Ошибка  " << err << " в L502_SetAdcFreq(" << freq << ")" << endl;
		else cout << "ADC_COLLECT_FREQ = " << ADC_COLLECT_FREQ << endl << "ADC_FRAME_FREQ = " << ADC_FRAME_FREQ << endl;
	}
	err = L502_AsyncGetAdcFrame(hnd, flags, tout, data.current_data);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_AsyncGetAdcFrame(" << flags << ")" << endl;
	return data.current_data;
}
double LCard::SingleDigitalRead() {
	cout << "Эта функция не написана, да и не особо нужна" << endl;
}
/// <summary>
/// Экстренный отвод иглы
/// </summary>
void LCard::BackstepZ() {
	double cur_h = cur_volt[0];
	for (double i = cur_h - 0.05; i > 0; i -= 0.00005) {
		SingleAnalogOut(i);
	}

	for (int k = 0; k < 6; k++) {
		SingleAnalogOut(5);
		for (double i = min((cur_h / 1.5 - 0.2 + k / 2), 5); i > 0; i -= 0.0003) {
			SingleAnalogOut(i);
		}
	}

}
/// <summary>
/// Чтение сигнала с платы. Возвращает обработанную информацию в data
/// </summary>
ADC_Collect LCard::AnalogRead(int timeout_ms , int bufsize ) {
	StartReadStream();
	
	int recv_zero_cnt = 0;
	err = L502_Recv(hnd, buf, bufsize, timeout_ms);
	while (err == 0) {
		err = L502_Recv(hnd, buf, bufsize, timeout_ms);
		recv_zero_cnt++;
		if (recv_zero_cnt > RECIVE_COUNT_TIMEOUT) {
			if (serial == serial_1)
				BackstepZ();
			SingleAnalogOut(0);
			cout << "Более " << RECIVE_COUNT_TIMEOUT << " раз отсутсвия принятых данных с платы" << endl;
			cerr << "Ошибка платы ввыода-вывода. Перезагрузите компьютер.";
			getchar(); getchar();
			exit(0);
		}
	}

	count_ADC_data = err;

	if (err < 0) cerr << "Ошибка  " << err << " в L502_Recv()" << endl;
	err = L502_ProcessAdcData(hnd, buf, data.current_data, &count_ADC_data, L502_PROC_FLAGS_VOLT);
	if (err == -140) {
		StopReadStream();
		StartReadStream();
		err = L502_Recv(hnd, buf, bufsize, timeout_ms);
		err = L502_ProcessAdcData(hnd, buf, data.current_data, &count_ADC_data, L502_PROC_FLAGS_VOLT);
	}
	else if ((err != 0) && (err != -11)) cerr << "Ошибка  " << err << " в L502_ProcessAdcData()" << endl;
	data.parse_channels();
	data.recv_cnt = count_ADC_data;
	return data;

}
void LCard::StopReadStream() {
	is_reading = 0;

	err = L502_StreamsStop(hnd);
	if (err != 0) cerr << "Ошибка  " << err << " в L502_StreamsStop()" << endl;
}
void LCard::StartReadStream() {
	if (is_reading == 0) {
		err = L502_StreamsStart(hnd);
		if (err != 0) {
			cerr << "Ошибка  " << err << " в L502_StreamsStart()" << endl;

		}
		else { is_reading = 1; }
	}
}
LCard::~LCard() {
	StopReadStream();
	err = L502_Close(hnd);
	if (err != 0) cerr << "Ошибка  " << err << "  в L502_Close()" << endl;
	err = L502_Free(hnd);
	if (err != 0) cerr << "Ошибка  " << err << "  в L502_Free()" << endl;
}

/*LCard(char* _serial) {
		serial = _serial;
		hnd = L502_Create();
		L502_Close(hnd);
		err = L502_Open(hnd, serial);
		if (err != 0) cerr << "Ошибка  " << err << "  в L502_Open()" << endl;
	}
	LCard(t_l502_hnd _hnd, char* _serial) {
		serial = _serial;
		hnd = _hnd;
		L502_Close(hnd);
		err = L502_Open(hnd, serial);
		if (err != 0) cerr << "Ошибка  " << err << "  в L502_Open()" << endl;
	}*/