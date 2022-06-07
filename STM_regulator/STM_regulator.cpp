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


int main()
{
    setlocale(LC_ALL, "Russian");

    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentProcess(), THREAD_PRIORITY_TIME_CRITICAL);
    cout << GetPriorityClass(GetCurrentProcess()) << endl;
    

    Regulator regul;
    //getchar(); getchar();

    ADC_Collect data = regul.XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
    for (int i = 0; i < regul.XYCard.data.ch_count; i++) {
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
    //regul.Retract(0,0.5,1);
    getchar(); getchar();
    //regul.Landing(4, 5, 0.12);
   
  
    cout << "PID started" << endl;
    //int volt = 0.55;
    //uwait(30000000);
    
    //regul.R_V_TransistorCalibration2();
    regul.R_NV_TransistorCalibration();

    regul.~Regulator();
    getchar(); getchar();
}
//Timer tmr;
//tmr.set_to_zero();
//while (true) {
//    regul.piezo.ZFJumpTo(1 + sin(tmr.get_full_interval() / 10000), regul.ZCard);
//}
// Запуск программы: CTRL+F5 или меню "Отладка" > "Запуск без отладки"
// Отладка программы: F5 или меню "Отладка" > "Запустить отладку"

// Советы по началу работы 
//   1. В окне обозревателя решений можно добавлять файлы и управлять ими.
//   2. В окне Team Explorer можно подключиться к системе управления версиями.
//   3. В окне "Выходные данные" можно просматривать выходные данные сборки и другие сообщения.
//   4. В окне "Список ошибок" можно просматривать ошибки.
//   5. Последовательно выберите пункты меню "Проект" > "Добавить новый элемент", чтобы создать файлы кода, или "Проект" > "Добавить существующий элемент", чтобы добавить в проект существующие файлы кода.
//   6. Чтобы снова открыть этот проект позже, выберите пункты меню "Файл" > "Открыть" > "Проект" и выберите SLN-файл.
