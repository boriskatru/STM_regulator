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
    std::cout.precision(4);

    Regulator regul;
    
    //getchar(); getchar();
    //////////////ПРОВЕРКА СИГНАЛА ПЛАТ///////////////////////
    ADC_Collect data = regul.XYCard.AnalogRead(100, ADC_BUF_SIZE_2);
    for (int i = 0; i < regul.XYCard.data.ch_count; i++) {
        cout << "LC ch " << i << " value: " << data.Average(8, i) << endl;
    }
    double data1 = regul.ZCard.SingleAnalogRead();
    cout << "NI ch value: " << data1 << endl;
    Timer tmr;
    string timestr = get_time_string();
    cout << endl << "Programm started..." << endl;
   // regul.R_CVg_TransistorCalibration();                              // To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC2) to C2; X4 to amplified C3
   // regul.Pn_CVg_TransistorCalibration(0.46, 0.62, 0.002);            // To calibrate Noise-V(gate) connect Z_coarse(NDAC1) to C1
    //getchar(); getchar();
   

    
    //regul.Retract(5,0.3,2);
 
    //regul.Landing(4, 4, 0.25);
   // regul.ZCard.BackstepZ();
    //return 0;
    //regul.rise();
    //regul.TouchScan(0.3, 0.15, 0.04, 4, 4, 0.01, 0.01, MIN_STEP_SIZE, 2);
    //return 0;
    int stop = 0;
    double  height = regul.IntPID_exp(0.5, 0.3, 240*1000000, 0);
    
    for (int count = 1; count <= 100; count++) {
        
        regul.XYCard.StartReadStream();
        height = regul.IntPID_exp(0.5, 0.3, 3*1000000, height);
        data = regul.XYCard.AnalogRead(0, ADC_BUF_SIZE_2);
        regul.XYCard.StopReadStream();
        data.print_f_VANC("VANC_" + to_string(count) + ".dat", "../../scans/" + timestr);
        cout << "done   " << count << " VANCS" << endl;
        //cin >> stop;
        //if (stop) break;
    }
    regul.MHome();
    //while (true) {
    //    regul.piezo.ZFJumpTo(2 * sin(tmr.get_full_interval() / 20000)+2, regul.ZCard);
    //    //regul.ZCard.SingleAnalogOut(2 * sin(tmr.get_full_interval() / 20000), 0U);
    //   
    // }
   
    /////////////////КОНЕЦ ТЕСТА//////////////////////////
    //for (int i = 0; i < 500; i++) {
    //    regul.ZStep(FORWARD,3);
    //    cout << i << endl;
    //    uwait(20000);
    //}
    //getchar(); getchar();
  

    //                   КАЛИБРОВКИ                    //
    //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
    //regul.Pn_CVg_TransistorCalibration(0.35,0.6,0.002);           // To calibrate Noise-V(gate) connect Z_coarse(NDAC1) to C1
   
    //regul.R_CVg_TransistorCalibration();                          // To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC2) to C2; X3 to amplified C3
    

  
   
   //текстовая строка с датой (для создания файлов и папок)

   

    //////////////СКРИПТ ДЛЯ КАЛИБРОВКИ ДЕТЕКТОРА МОЩНОСТИ////////////////
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
    cout << endl << "Programm finished..." << endl;
    regul.~Regulator();
    getchar(); getchar();
}
