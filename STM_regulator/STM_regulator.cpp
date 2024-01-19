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
    cout << "XYCard.data.ch_count" << regul.XYCard.data.ch_count << endl;
    for (int i = 0; i < regul.XYCard.data.ch_count; i++) {
        cout << "LC ch " << i << " value: " << data.Average(16, i) << endl;
    }
    regul.XYCard.StopReadStream();
    double data1 = regul.ZCard.SingleAnalogRead();
    cout << "NI ch value: " << data1 << endl;

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
                uwait(350000);                 
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


//                   Перемещения                   //
//////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
//regul.Retract(10, 0.3, 2);
//regul.StepXY(-10, 10);
//regul.ZStep(FORWARD);
//regul.Landing(2, 5, 0.25);
//regul.ZCard.BackstepZ();
//regul.piezo.MoveTo(Vecter(1.0, 1.0, 0), 100, MIN_STEP_SIZE, regul.ZCard, regul.XYCard);


//                   Сканы и измерения                    //
//////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
//regul.TouchScan(0.3, 0.15, 0.08, 4, 4, 0.01, 0.01, MIN_STEP_SIZE, 2);
//regul.CapStepScan(5, 5000, 0.2, 50, 50, 1, 1);
//regul.CapScan(5, 5000, 0.25, 2000, 0, 0.1, 5, 0, 0.1, 5, 3.5);
//regul.ConstHScan(0.2, 0.2, 2, 2000, 0 * MIN_STEP_SIZE, 20 * MIN_STEP_SIZE, 2000 * MIN_STEP_SIZE, 0 * MIN_STEP_SIZE, 20 * MIN_STEP_SIZE, 2000 * MIN_STEP_SIZE, MIN_STEP_SIZE, 0.05, 200);
//regul.VANC_PID(20);




//                   КАЛИБРОВКИ                     //
//////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
//regul.Pn_CVg_TransistorCalibration(0.35,0.6,0.002);           // To calibrate Noise-V(gate) connect Z_coarse(NDAC1) to C1
//regul.R_CVg_TransistorCalibration();                          // To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC2) to C2; X3 to amplified C3
