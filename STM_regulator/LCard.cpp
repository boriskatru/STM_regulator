#include <fstream>
#include "LCard.h"

using namespace std;

ADC_Collect::ADC_Collect(int ch_count, int ADC_BUF_SIZE ) :
	average(ch_count, 0),
	ch_count(ch_count),
	s_ch_bufsz(ADC_BUF_SIZE / ch_count),
	err_cnt(0),
	input(ch_count, vector<double>(ADC_BUF_SIZE / ch_count, 0))/*, swap(0)*/ {
	current_data = (double*)calloc( ADC_BUF_SIZE, sizeof(double));
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
	 cout << recv_cnt<< endl;
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

LCard::LCard(int card_No, int ADC_CH_COUNT, int ADC_BUF_SIZE) : Card(ADC_BUF_SIZE, ADC_CH_COUNT, "LCard", 1), data(ADC_CH_COUNT, ADC_BUF_SIZE), next_lch(0) {
	
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
		for (int i = 0; i < get_list_res; i++) {
			cout << "serial_" << i <<" =   " << serial_list[0] << endl;
			/*getchar();getchar();*/
		}

	}
	hnd = L502_Create();

	serial = serial_list[card_No - 1];

	cout << "Card" << card_No << "  have serial  " << serial << endl;
	error = L502_Close(hnd);
	if (error != 0) cerr << "Ошибка  " << error << "  в L502_Close()" << endl;
	error = L502_Open(hnd, serial);
	if (error != 0) cerr << "Ошибка  " << error << "  в L502_Open()" << endl;

	error = L502_SetLChannelCount(hnd, ADC_CH_COUNT);
	for (int i = 0; i < ADC_CH_COUNT; i++) {
		if (!error)
		{
			/* первый логический канал соответствует измерению 1 канала
			относительно общей земли */
			error = L502_SetLChannel(hnd, i, i, L502_LCH_MODE_COMM, L502_ADC_RANGE_10, 0);
			if (error)
				cout << "Ошибка в L502_SetLChannel()" << error << endl;
		}
	}

	error = L502_AsyncOutDac(hnd, L502_DAC_CH1, 0.0, 0x0001) + L502_AsyncOutDac(hnd, L502_DAC_CH2, 0.0, 0x0001);
	while (error != 0) {
		cerr << "Ошибка  " << error  << "  в L502_AsyncOutDac()" << endl;
		error = L502_Close(hnd);
		if (error != 0) cerr << "Ошибка  " << error << "  в L502_Close()" << endl;
		error = L502_Open(hnd, serial);
		if (error != 0) cerr << "Ошибка  " << error << "  в L502_Open()" << endl;
		error = L502_AsyncOutDac(hnd, L502_DAC_CH1, 0.0, 0x0001) + L502_AsyncOutDac(hnd, L502_DAC_CH2, 0.0, 0x0001);
	}
	error = L502_SetAdcFreq(hnd, &ADC_COLLECT_FREQ, &ADC_FRAME_FREQ);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_SetAdcFreq()" << endl;
	else cout << "ADC_COLLECT_FREQ = " << ADC_COLLECT_FREQ << endl << "ADC_FRAME_FREQ = " << ADC_FRAME_FREQ << endl;

	error = L502_StreamsDisable(hnd, L502_STREAM_DIN);
	error = L502_StreamsDisable(hnd, L502_STREAM_DOUT);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_StreamsDisable()" << endl;
	error = L502_StreamsEnable(hnd, L502_STREAM_ADC);
	error = L502_StreamsEnable(hnd, L502_STREAM_DAC1);
	error = L502_StreamsEnable(hnd, L502_STREAM_DAC2);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_StreamsEnable()" << endl;
	L502_SetDmaIrqStep(hnd, L502_DMA_CH_IN, 9000000);
	L502_SetDmaIrqStep(hnd, L502_DMA_CH_OUT, 9000000);
	L502_SetDmaBufSize(hnd, L502_DMA_CH_IN, ADC_BUF_SIZE);
	L502_SetDmaBufSize(hnd, L502_DMA_CH_OUT, 16);
	L502_Configure(hnd, 0);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_Configure()" << endl;
	
	
}
/// <summary>
/// Установка настроек платы (см документацию к плате, необходимо отправлять нужный флаг)
/// </summary>
/// <param name="flags"> флаг настройки </param>
/// 
void LCard::SetMode(uint32_t flags) {
	error = L502_Configure(hnd, flags);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_Configure(" << flags << ")" << endl;
}
/// <summary>
/// Единичный вывод на канал DAC
/// </summary>
/// <param name="data"> Задаваемый сигнал напряжения</param>
/// <param name="channel"> Номер канала DAC </param>
/// <param name="flags"></param>
void LCard::SingleAnalogOut(double data, unsigned int channel , double timeout ) {
	error = L502_AsyncOutDac(hnd, channel, data, ANALOG_OUT_FLAG);
	cur_volt[channel] = data;
	while (error != 0) {
		cerr << "Ошибка  " << error << "  в L502_AsyncOutDac()" << endl;
		error = L502_Close(hnd);
		if (error != 0) cerr << "Ошибка  " << error << "  в L502_Close()" << endl;
		error = L502_Open(hnd, serial);
		if (error != 0) cerr << "Ошибка  " << error << "  в L502_Open()" << endl;
		error = L502_AsyncOutDac(hnd, channel, data, ANALOG_OUT_FLAG);
	}
}
void LCard::SingleDigitalOut(uint32_t val, uint32_t mask) {
	error = L502_AsyncOutDig(hnd, val, mask);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_AsyncOutDig(" << mask << ")" << endl;
}
double LCard::SingleAnalogRead(int channel, double freq , uint32_t tout , uint32_t flags ) {
	if ((freq != -1) && (freq != ADC_COLLECT_FREQ)) {
		ADC_COLLECT_FREQ = freq;
		ADC_FRAME_FREQ = freq / 2;
		error = L502_SetAdcFreq(hnd, &ADC_COLLECT_FREQ,
			&ADC_FRAME_FREQ);
		if (error != 0) cerr << "Ошибка  " << error << " в L502_SetAdcFreq(" << freq << ")" << endl;
		else cout << "ADC_COLLECT_FREQ = " << ADC_COLLECT_FREQ << endl << "ADC_FRAME_FREQ = " << ADC_FRAME_FREQ << endl;
		//getchar(); getchar();
	}
	error = L502_AsyncGetAdcFrame(hnd, flags, tout, data.current_data);
	read_cnt++;
	//if (error != 0) cerr << "Ошибка  " << error << " в L502_AsyncGetAdcFrame(" << flags << ")" << endl;
	return data.current_data[channel];
}
double LCard::SingleAnalogRead(int channel, double timeout)
{
	return SingleAnalogRead(channel, -1, 1U, 1U);
}
double* LCard::AsyncAnalogRead(double freq , uint32_t tout , uint32_t flags ) {
	if ((freq != -1) && (freq != ADC_COLLECT_FREQ)) {
		ADC_COLLECT_FREQ = freq;
		ADC_FRAME_FREQ = freq / 2;
		error = L502_SetAdcFreq(hnd, &ADC_COLLECT_FREQ,
			&ADC_FRAME_FREQ);
		if (error != 0) cerr << "Ошибка  " << error << " в L502_SetAdcFreq(" << freq << ")" << endl;
		else cout << "ADC_COLLECT_FREQ = " << ADC_COLLECT_FREQ << endl << "ADC_FRAME_FREQ = " << ADC_FRAME_FREQ << endl;
	}
	error = L502_AsyncGetAdcFrame(hnd, flags, tout, data.current_data);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_AsyncGetAdcFrame(" << flags << ")" << endl;
	return data.current_data;
}
double LCard::SingleDigitalRead() {
	cout << "Эта функция не написана, да и не особо нужна" << endl;
	return 0;
}

/// <summary>
/// Чтение сигнала с платы. Возвращает обработанную информацию в data
/// </summary>
ADC_Collect LCard::AnalogRead(int timeout_ms , int bufsize ) {
	StartReadStream();
	
	int recv_zero_cnt = 0;
	error = L502_Recv(hnd, buf, bufsize, timeout_ms);
	while (error == 0) {
		error = L502_Recv(hnd, buf, bufsize, timeout_ms);
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

	count_ADC_data = error;

	if (error < 0) cerr << "Ошибка  " << error << " в L502_Recv()" << endl;
	error = L502_ProcessAdcData(hnd, buf, data.current_data, &count_ADC_data, L502_PROC_FLAGS_VOLT);
	if (error == -140) {
		StopReadStream();
		StartReadStream();
		error = L502_Recv(hnd, buf, bufsize, timeout_ms);
		error = L502_ProcessAdcData(hnd, buf, data.current_data, &count_ADC_data, L502_PROC_FLAGS_VOLT);
	}
	else if ((error != 0) && (error != -11)) cerr << "Ошибка  " << error << " в L502_ProcessAdcData()" << endl;
	data.recv_cnt = count_ADC_data;
	data.parse_channels();	
	return data;

}
void LCard::StopReadStream() {
	is_reading = 0;

	error = L502_StreamsStop(hnd);
	if (error != 0) cerr << "Ошибка  " << error << " в L502_StreamsStop()" << endl;
}
void LCard::StartReadStream() {
	if (is_reading == 0) {
		error = L502_StreamsStart(hnd);
		if (error != 0) {
			cerr << "Ошибка  " << error << " в L502_StreamsStart()" << endl;

		}
		else { is_reading = 1; }
	}
}
void LCard::FullStop()
{
	StopReadStream();
	SingleAnalogOut(0, 0U);
	SingleAnalogOut(0, 1U);
}
LCard::~LCard() {
	FullStop();
	error = L502_Close(hnd);
	if (error != 0) cerr << "Ошибка  " << error << "  в L502_Close()" << endl;
	error = L502_Free(hnd);
	if (error != 0) cerr << "Ошибка  " << error << "  в L502_Free()" << endl;
}

