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
    regul.XYCard.StopReadStream();
    double data1 = regul.ZCard.SingleAnalogRead();
    cout << "NI ch value: " << data1 << endl;

    /////////////////КОНЕЦ ТЕСТА//////////////////////////

    Timer tmr;
    string timestr = get_time_string();
    cout << endl << "Programm started..." << endl;
    //regul.R_CVg_TransistorCalibration(0.002,0.47,0.65,0.07);                              // To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC2) to C2; X4 to amplified C3
    //regul.Pn_CVg_TransistorCalibration(0.43, 0.66, 0.002);            // To calibrate Noise-V(gate) connect Z_coarse(NDAC1) to C1
    //getchar(); getchar();
    //regul.StepXY(30, -10);
   // regul.Landing(2, 5, 0.2);
    //regul.piezo.MoveTo(Vecter(3.0, 1.5, 0), 100, MIN_STEP_SIZE, regul.ZCard, regul.XYCard);
    //regul.piezo.MoveTo(Vecter(0.0, 0.0, 0),100, MIN_STEP_SIZE,regul.ZCard,regul.XYCard);
    //double  height = regul.IntPID_exp(0.5, 0.3, 20 * 1000000, 0,-1);
    //regul.MHome(); 
    //regul.ConstH_Scan(0.3,0.5,2,0.01,0.01,2e-4,2e-4,1.5e-4,0);
    //regul.Retract(5,0.3,2);
    //getchar(); getchar();
    //regul.Landing(2, 5, 0.35);
    //getchar(); getchar();
    //regul.ZCard.BackstepZ();
    //regul.rise();
    //regul.ConstH_Scan(0.1, 0.2, 2, 2000, 2000*MIN_STEP_SIZE, 2000*MIN_STEP_SIZE, 5*MIN_STEP_SIZE, 5*MIN_STEP_SIZE, MIN_STEP_SIZE,0,200);
    //regul.TouchScan(1, 0.15, 0.12, 5, 5, 0.025, 0.025, MIN_STEP_SIZE, 2, 200);
    //regul.VANC_PID(50, 0.3, 0.2, 240);
   
    return 0;
      







   //                   Перемещения                   //
   //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
   //regul.Retract(10, 0.3, 2);
   //regul.StepXY(-10, 10);
   //regul.ZStep(FORWARD);
   //regul.Landing(2, 5, 0.25);
   // regul.ZCard.BackstepZ();

     

 
   //                   Сканы и измерения                    //
   //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
   //regul.TouchScan(0.3, 0.15, 0.08, 4, 4, 0.01, 0.01, MIN_STEP_SIZE, 2);
   //regul.VANC_PID(20);





    //                   КАЛИБРОВКИ                    //
    //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
    //regul.Pn_CVg_TransistorCalibration(0.35,0.6,0.002);           // To calibrate Noise-V(gate) connect Z_coarse(NDAC1) to C1
   
    //regul.R_CVg_TransistorCalibration();                          // To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC2) to C2; X3 to amplified C3
    



   

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
   /*double  height = regul.IntPID_exp(0.2, 0.05 , 100*1000000, 0);
    
    for (int count = 1; count <= 20; count++) {
        tmr.get_loop_interval();
        regul.XYCard.StartReadStream();
        height = regul.IntPID_exp(0.2, 0.05, 1*1000000, height);
        data = regul.XYCard.AnalogRead(0, ADC_BUF_SIZE_2);
        regul.XYCard.StopReadStream();
        data.print_f_VANC("VANC_" + to_string(count) ,".bin", "../../scans/" + timestr);
        cout << "done   " << count << " VANCS" << endl;
        //cin >> stop;
        //if (stop) break;
        cout << "loop time : " << tmr.get_loop_interval()/1000000 << endl;
    }
    regul.MHome();*/
   //////////////КОНЕЦ СКРИПТА//////////////////
    cout << endl << "Programm finished..." << endl;
    regul.~Regulator();
    getchar(); getchar();
}
