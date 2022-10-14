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
    //////////ПРОВЕРКА СИГНАЛА ПЛАТ//////////////////////////
    ADC_Collect data = regul.XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
    for (int i = 0; i < regul.XYCard.data.ch_count; i++) {
        cout << "ch " << i << " value: " << data.Average(8, i) << endl;
    }
    double data1 = regul.ZCard.SingleAnalogRead();
    for (int i = 0; i < 1; i++) {
        cout << "N0 " << i << " value: " << regul.ZCard.SingleAnalogRead()<< endl;
    }

    /////////////////КОНЕЦ ТЕСТА//////////////////////////
    for (int i = 0; i < 500; i++) {
        regul.ZStep(FORWARD,3);
        cout << i << endl;
        uwait(20000);
    }
    //getchar(); getchar();
    //regul.R_NV_TransistorCalibration(0.39,0.6);
   // regul.R_V_TransistorCalibration(0.003, 0.36, 0.6);
    //regul.Retract(20,0.5,1);

    getchar(); getchar();
   
   string timestr = get_time_string(); //текстовая строка с датой (для создания файлов и папок)



    //////////СКРИПТ ДЛЯ КАЛИБРОВКИ ДЕТЕКТОРА МОЩНОСТИ////////////////
  /* string timestr = get_time_string();
   ofstream file;
   file.open("calibration_detector.txt", std::ofstream::out);
   for (int i = 0; i < 140; i++)
   {
       cout << "Expecting" << -i * 0.1 << "dBm / Press Enter" << endl;
       getchar();
       regul.XYCard.AnalogRead(ADC_BUF_SIZE_2 / 20, ADC_BUF_SIZE_2 / 100);
       data = regul.XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2, ADC_BUF_SIZE_2);
       regul.XYCard.StopReadStream();
       file << data.Average(ADC_BUF_SIZE_2 / 4, 3) << "  " << -i * 0.1 << endl;
       cout << i << " point written" << endl;


   }*/
   //////////////КОНЕЦ СКРИПТА//////////////////




   ////////////////////СКРИПТ ДЛЯ ИЗМЕРЕНИЯ СЕРИИ ВАХ В ОДНОЙ ТОЧКЕ//////////////////////////////
   /*data = regul.XYCard.AnalogRead(ADC_BUF_SIZE_2 / 20, ADC_BUF_SIZE_2/100);
   regul.XYCard.StopReadStream();
   data.print_f("VANC_test.dat", "../../scans/" + timestr);
   cout << timestr << endl;
   cout << "PID started" << endl;
   double height =  regul.IntPID_exp(0.5, 0.6, 300000000, 0);
   
   for (int i = 0; i < 20; i++)
   {    
       regul.XYCard.StartReadStream();       
       regul.ZCard.StartReadStream();              
       height = regul.IntPID_exp(0.3, 0.6, 20000000, height);
       data = regul.XYCard.AnalogRead(ADC_BUF_SIZE_2 / 2, ADC_BUF_SIZE_2);
       regul.XYCard.StopReadStream();
       regul.ZCard.StopReadStream();
       data.print_f("VANC_" + to_string(i) + ".dat", "../../scans/" + timestr);
       cout << "done   " << i << "VANCS" << endl;
   }*/
   //////////////КОНЕЦ СКРИПТА//////////////////

    regul.~Regulator();
    getchar(); getchar();
}

// Запуск программы: CTRL+F5 или меню "Отладка" > "Запуск без отладки"
// Отладка программы: F5 или меню "Отладка" > "Запустить отладку"

// Советы по началу работы 
//   1. В окне обозревателя решений можно добавлять файлы и управлять ими.
//   2. В окне Team Explorer можно подключиться к системе управления версиями.
//   3. В окне "Выходные данные" можно просматривать выходные данные сборки и другие сообщения.
//   4. В окне "Список ошибок" можно просматривать ошибки.
//   5. Последовательно выберите пункты меню "Проект" > "Добавить новый элемент", чтобы создать файлы кода, или "Проект" > "Добавить существующий элемент", чтобы добавить в проект существующие файлы кода.
//   6. Чтобы снова открыть этот проект позже, выберите пункты меню "Файл" > "Открыть" > "Проект" и выберите SLN-файл.
