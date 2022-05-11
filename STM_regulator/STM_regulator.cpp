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
using namespace std::chrono;
int main()
{
    setlocale(LC_ALL, "Russian");

    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentProcess(), THREAD_PRIORITY_TIME_CRITICAL);
    cout << GetPriorityClass(GetCurrentProcess()) << endl;


    Regulator regul;


    ADC_Collect data = regul.XYCard.AnalogRead(0, ADC_BUF_SIZE_2);
    for (int i = 0; i < 3; i++) {
        cout << "ch " << i << " value: " << data.Average(8, i) << endl;
    }
    ADC_Collect data1 = regul.ZCard.AnalogRead();
    for (int i = 0; i < 1; i++) {
        cout << "N0 " << i << " value: " << regul.ZCard.AnalogRead().current_data[0] << endl;
        cout << "N1 " << i << " value: " << regul.ZCard.AnalogRead().current_data[1] << endl;
        cout << "N2 " << i << " value: " << regul.ZCard.AnalogRead().current_data[2] << endl;
        cout << "N3 " << i << " value: " << regul.ZCard.AnalogRead().current_data[3] << endl;
    }
    //getchar(); getchar();
    regul.Retract(3);
    //regul.Landing(4, 5, 0.09);
    getchar(); getchar();

    // regul.rise(3, 0.25);
     //regul.XYCard.AnalogRead(0, ADC_BUF_SIZE_2).show();
    cout << "PID started" << endl;
    //int volt = 0.55;
    //uwait(30000000);


    regul.XYCard.StopReadStream();
    double tmp = regul.IntPID_exp(1, 0.1, 100000000, 0);
    regul.XYCard.StartReadStream();
    for (int i = 0; i < 30; i++) {
        tmp = regul.IntPID_exp(1, 0.1, 20000000, tmp);
        data = regul.XYCard.AnalogRead(0, ADC_BUF_SIZE_2);
        cout << endl << endl << i << endl;
        cout << " printing data..." << endl;
        data.print_f("VAC" + to_string(i) + ".dat");
        cout << "data printed in file VAC" << i << ".dat " << endl;
    }
    regul.IntPID_exp(1, 0.07, 0, tmp);


    //regul.TouchScan(1,0.2,-0.01,5,5,0.05,0.05);

    regul.~Regulator();
    getchar(); getchar();
}