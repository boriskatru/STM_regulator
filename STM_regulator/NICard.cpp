#include "NICard.h"
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
	int32       error = 0;
	char        errBuff[2048] = { '\0' };


	int32       read_cnt = 0;
	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	//DAQmxReadAnalogF64(taskHandle, 1, 5.0, DAQmx_Val_GroupByChannel, data_, N_, &read, NULL);
	
	//DAQmxWriteAnalogF64(taskHandle, N_, 0, 1.0, DAQmx_Val_GroupByChannel, data_, NULL, NULL);
	/*for (int i = 0; i < 10; i++) {
		cout << data_[i*9] << "  ";
	}
	cout << endl;*/


	return 0;
}
int32 CVICALLBACK EveryNCallback2(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
	int32       error = 0;
	char        errBuff[2048] = { '\0' };
	int32       read_cnt = 0;


	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	//DAQmxWriteAnalogF64(taskHandle, N_, 0, 0.4, DAQmx_Val_GroupByChannel, data_, NULL, NULL);

	return 0;
}
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData)
{
	int32   error = 0;
	char    errBuff[2048] = { '\0' };

	// Check to see if an error stopped the task.
	if (DAQmxFailed(error = (status))) goto Error; else;


Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}
	return 0;
}

NICard::NICard() : Card(1, 1, "NICard", 1) {

	PTR_ = ADC_BUF_SIZE_1;
	data_ =  (double*)calloc(PTR_, sizeof(double));;
	error = 0;
	ai_0 = 0;
	ao_0 = 0;
	ao_1 = 0;
	
	/*********************************************/
	 //DAQmx Configure Code
	/*********************************************/
	DAQmxCreateTask("", &ai_0); // создаём задачу считывания сигнала с канала N0
	DAQmxCreateTask("", &ao_0); // создаём вывода сигнала на канал NDAC
	

	DAQmxCreateAIVoltageChan(ai_0, "Dev1/ai0", "", DAQmx_Val_RSE, -5.0, 5.0, DAQmx_Val_Volts, NULL);	// настраиваем задачу считывания
	DAQmxCfgSampClkTiming(ai_0, "", 250000.0, DAQmx_Val_Rising, DAQmx_Val_HWTimedSinglePoint, PTR_);
	DAQmxRegisterDoneEvent(ai_0, 0, DoneCallback, NULL);


	DAQmxCreateAOVoltageChan(ao_0, "Dev1/ao0:1", "", -5.0, 5.0, DAQmx_Val_Volts, NULL);					// настраиваем задачу вывода  NDAC
	DAQmxCfgSampClkTiming(ao_0, "", 900000.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, PTW_);


	
	is_writing[0] = 0;
}

NICard::~NICard()
{
	FullStop();
}

void NICard::SetMode(uint32_t flags)
{
}

void NICard::SingleAnalogOut(double signal, unsigned int channel , double timeout  ) {

	if (channel) {
		if (!is_writing[1]) {
			DAQmxStartTask(ao_0);
			//DAQmxStopTask(ao_0);
			//is_writing[0] = 0;
			//DAQmxStartTask(ao_1);
			is_writing[1] = 1;
		}
		double dat[2] = { cur_volt[0], signal };
		//cout << "!!" << endl;
		DAQmxWriteAnalogF64(ao_0, 1,autostart, timeout, 1, dat , NULL, NULL);
		//(ao_1, autostart, timeout, signal, NULL);
		cur_volt[1] = signal;
		
	}
	else {
		
		if (!is_writing[0]) {
			//DAQmxStopTask(ao_1);
			//is_writing[1] = 0;
			DAQmxStartTask(ao_0);
			is_writing[0] = 1;
		}
		double dat[2] = { signal, cur_volt[1]};
		//cout << endl;
		DAQmxWriteAnalogF64(ao_0, 1, autostart, timeout, 1, dat, NULL, NULL);
		 //DAQmxWriteAnalogScalarF64(ao_0, autostart, timeout, signal, NULL);
		cur_volt[0] = signal;
		
	}
}
void NICard::SingleAnalogOut(double* signal) {

	
	if (!is_writing[1]) {
		DAQmxStartTask(ao_0);
		//DAQmxStopTask(ao_0);
		//is_writing[0] = 0;
		//DAQmxStartTask(ao_1);
		is_writing[1] = 1;
	}
		
	//cout << "!!" << endl;
	DAQmxWriteAnalogF64(ao_0, 1, autostart, 0.1, 1, signal, NULL, NULL);
	//(ao_1, autostart, timeout, signal, NULL);
	cur_volt[1] = signal[1];
	cur_volt[0] = signal[0];
	
	
}
double NICard::SingleAnalogRead(int channel, double timeout) {
	if (!is_reading) {
		StartReadStream();
	}
	
	while (DAQmxReadAnalogScalarF64(ai_0, timeout, data_, NULL) != 0) {
		DAQmxStopTask(ai_0);
		DAQmxStartTask(ai_0);
		
	}
	read_cnt ++;
	//cout << data_[0] <<  endl;
	return data_[channel];
	
}

void NICard::SingleDigitalOut(uint32_t val, uint32_t mask)
{
}
double NICard::SingleDigitalRead()
{
	return 0.0;
}

void NICard::StopDAC(int channel)
{
	if (channel) {
		DAQmxWriteAnalogScalarF64(ao_1, 0, 1, 0, NULL);
		is_writing[1] = 0;
		DAQmxStopTask(ao_1);

	}
	else {
		DAQmxWriteAnalogScalarF64(ao_0, 0, 1, 0, NULL);
		is_writing[0] = 0;
		DAQmxStopTask(ao_0);
	}
}
void NICard::StartDAC(int channel)
{
	if (channel) {
		DAQmxStartTask(ao_1);
		is_writing[1] = 1;

	}
	else {
		DAQmxStartTask(ao_0);
		is_writing[0] = 1;
	}
}

void NICard::StopReadStream()
{
	is_reading = 0;
	DAQmxStopTask(ai_0);
	
}
void NICard::StartReadStream()
{
	DAQmxStartTask(ai_0);
	is_reading = 1;
}

void NICard::FullStop() {
	read_cnt = 0;
	is_reading = 0;
	is_writing[0] = is_writing[1]= 1;
	DAQmxStopTask(ai_0);
	DAQmxStopTask(ao_0);
	DAQmxStopTask(ao_1);
}
