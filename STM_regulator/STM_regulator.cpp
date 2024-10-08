// STM_regulator.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <chrono>
#include <l502api.h>
#include "wait_bh.h"
#include "piezo_positioners.h"
#include "LCard.h"
#include "regulator.h"

using namespace std;
//using namespace std::chrono;

enum class Operation{Landing,Retract,Steps,Move,MHome,VANC,TouchScan,CapStepScan,Calibration_N_V, Calibration_R_V, Exit, Waiting};
Operation LoadOpeartion(string path = MAIN_FOLDER + COMMAND_STATUS_FILE) {
    fstream file;
    file.open(path, std::ios::in);
    int input;
    file >> input;
    file.close();
    return static_cast<Operation>(input);
}
void FinishOperation(string path = MAIN_FOLDER + COMMAND_STATUS_FILE) {
    fstream file;
    file.open(path, std::ios::out);
    file << static_cast<int>(Operation::Waiting);
    file.close();  
}


int main()
{
    setlocale(LC_ALL, "Russian");
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentProcess(), THREAD_PRIORITY_TIME_CRITICAL);
    cout << GetPriorityClass(GetCurrentProcess()) << endl;
    std::cout.precision(4);

    Regulator regul;
      
    //////////////ПРОВЕРКА СИГНАЛА ПЛАТ///////////////////////

    ADC_Collect data = regul.XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
    cout << "XYCard.data.ch_count" << regul.XYCard.ADC_CH_COUNT << endl;
    for (int i = 0; i < regul.XYCard.ADC_CH_COUNT; i++) {
        cout << "LC ch " << i << " value: " << data.Average(16, i) << endl;
    }
    regul.XYCard.StopReadStream();
    double data1 = regul.ZCard.SingleAnalogRead();
    cout << "NI ch value: " << data1 << endl << endl;

    /////////////////КОНЕЦ ТЕСТА//////////////////////////
    Timer tmr;
    string timestr = get_time_string();
    cout << endl << "Programm started..." << endl;

    Operation command = Operation::Waiting;
    while (1) {
        command = LoadOpeartion();
        switch (command) {
            case Operation::Landing:
                regul.Landing();
                FinishOperation();
                break;
            case Operation::Retract:
                regul.Retract();
                FinishOperation();
                break;
            case Operation::Steps:
                regul.StepXY();     
                FinishOperation();
                break;
            case Operation::Move:
                regul.MoveTo();    
                FinishOperation();
                break;
            case Operation::MHome:
                regul.MHome(DEFAULT_MICROSTEP_SIZE, true);
                FinishOperation();
                break;
            case Operation::VANC:
                regul.VANC_PID();
                FinishOperation();
                break;
            case Operation::TouchScan:
                regul.TouchScan();
                FinishOperation();
                break;
            case Operation::CapStepScan:
                regul.CapStepScan();
                FinishOperation();
                break;
            case Operation::Calibration_N_V:
                regul.Pn_CVg_TransistorCalibration();//  !!!!!!! TODO!!!!!!!!!
                FinishOperation();
                break;
            case Operation::Calibration_R_V:
                regul.R_CVg_TransistorCalibration();//  !!!!!!! TODO!!!!!!!!!
                FinishOperation();
                break;
            case Operation::Waiting:
                Sleep(350);              
                break;
            case Operation::Exit:
                cout << endl << "Programm finished..." << endl;
                regul.~Regulator();
                exit(0);
            default:
                cout << endl << "ERROR: Unrecognized command" << endl;
                FinishOperation();
                break;
        }
        
       
    }

    cout << endl << "Programm finished..." << endl;
    regul.~Regulator();
    return 0;
}

