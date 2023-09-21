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
    //getchar(); getchar();
    //regul.R_CVg_TransistorCalibration(0.002, 0.45, 0.6, 0.06, 4.0);            // To calibrate R-V(gate) connect Z_coarse to C1 ; Z_fine (NDAC2) to C2; X4 to amplified C3
    //regul.Pn_CVg_TransistorCalibration(0.44, 0.6, 0.001);                        // To calibrate Noise-V(gate) connect Z_coarse(NDAC1) to C1
    //getchar(); getchar();
    //regul.StepXY(50, -200);
    //regul.Retract(100, 1.0, 1);
    //regul.StepXY(-20, -20);
    //regul.Landing(5, 4.5, 0.08, 0, MIN_STEP_SIZE/50);
    //regul.StepXY(-1, 0, 1, 4);
    //getchar(); getchar();
 
    //regul.TouchScan(0.6, 0.08, 0.06, 0, 0.025, 5, 0, 0.025, 5, 1.6, MIN_STEP_SIZE, 3, 150);
    //regul.ConstHScan(0.5, 0.06, 3.0, 2000, 1000 * MIN_STEP_SIZE, 10 * MIN_STEP_SIZE, 2000 * MIN_STEP_SIZE, 1000 * MIN_STEP_SIZE, 10 * MIN_STEP_SIZE, 2000 * MIN_STEP_SIZE, MIN_STEP_SIZE, 0.1, 100);
 
    regul.piezo.MoveTo(Vecter(0.5, 0.0, 0.0), 5, MIN_STEP_SIZE / 20, regul.ZCard, regul.XYCard);
    regul.VANC_PID(2000, 0.4, 0.1, 180);
   /*getchar(); getchar();*/
    //regul.Landing(5, 5, 0.06, 0, MIN_STEP_SIZE/2);
   // regul.Retract(15, 0.3, 1);
   //
   //regul.CapStepScan(5, 5000, 0.4, 40, 20, 1, 0);
   //getchar(); getchar();
  
   //regul.CapStepScan(5, 5000, 0.4, 50, 50, 2, 2);


   // regul.CapScan(5, 5000, 0.25, 2000, 0, 0.1, 5, 0, 0.1, 5, 3.5);
   // regul.ConstHScan(0.2, 0.2, 2, 2000, 0 * MIN_STEP_SIZE, 20 * MIN_STEP_SIZE, 2000 * MIN_STEP_SIZE, 0 * MIN_STEP_SIZE, 20 * MIN_STEP_SIZE, 2000 * MIN_STEP_SIZE, MIN_STEP_SIZE, 0.05, 200);
   // regul.Pn_CVg_TransistorCalibration(0.35, 0.6, 0.002);           
   // regul.R_CVg_TransistorCalibration();                         



    return 0;
    //regul.ZStep(FORWARD);
   // regul.StepXY(80,0);
    //getchar(); getchar();
    //regul.TouchScan(0.4, 0.08, 0.08, 0, 0.05, 5, 0, 0.05, 5, 0.15, MIN_STEP_SIZE, 2, 200);
 
    //regul.StepXY(-10, 5);
    

    
    //regul.StepXY(0, 1, true, 4.5);
    //regul.piezo.MoveTo(Vecter(0.0, 5.0, 0.0), 5, MIN_STEP_SIZE / 20, regul.ZCard, regul.XYCard);
    //regul.Landing(4.5, 4.3, 0.15, 0, MIN_STEP_SIZE / 2);
   //regul.StepXY(0, 6);
    //regul.StepXY(-5, 0);
    //regul.StepXY(0, 2);regul.StepXY(2, 0);
    /*regul.StepXY(0, 1);
    for (int count = 1; count <= 12; count++) {   
      regul.StepXY(2, 0);
      regul.TouchScan( 0.08, 0.1, 5, -0.05, 0, 0, 0.05, 5, 0.25, MIN_STEP_SIZE, 2, 100, 0.6);     
      getchar(); getchar();
      
     
    }*/
   //regul.TouchScan(0.4, 0.08, 0.1, 0, 0.05, 5, 5, -0.05, 0, 0.25, MIN_STEP_SIZE / 2, 2, 100);
   
   
    //regul.Landing(5, 5, 0.15, 0, MIN_STEP_SIZE/2);
   
   // 
    //getchar(); getchar();
   
    //regul.piezo.MoveTo(Vecter(0.0, 0.0, 2.0), 10, MIN_STEP_SIZE / 20, regul.ZCard, regul.XYCard);
    //uwait(2000000);
    //regul.StepXY(16, 18);
    //regul.Landing(5, 5, 0.15, 0, MIN_STEP_SIZE / 2);
  
    //regul.CapStepScan(5, 5000, 0.2, 50, 50, 1, 1);

  
  //сканы с перемещеием по у между ними

  //for (int count = 1; count <= 20; count++) {
    //regul.piezo.MoveTo(Vecter(0.0, 0.0, 3.5), 100, MIN_STEP_SIZE / 20, regul.ZCard, regul.XYCard);
     // uwait(2000000);
      
       //regul.StepXY(0, 1, true, 5);
  //}

 
    
    //regul.piezo.MoveTo(Vecter(0.0, 0.0, 1.0),100, MIN_STEP_SIZE/20,regul.ZCard,regul.XYCard);
    //uwait(2000000);
    //double  height = regul.IntPID_exp(0.5, 0.3, 20 * 1000000, 0,-1);
    //regul.MHome(); 
    
    //regul.Retract(5,0.3,1);
 
    //getchar(); getchar();
    //regul.Landing(1, 4.3, 0.12);
    //getchar(); getchar();  
    //regul.ZCard.BackstepZ();
    //regul.rise();
   
    //regul.piezo.MoveTo(Vecter(2.2, 1.0, 0), 100, MIN_STEP_SIZE, regul.ZCard, regul.XYCard);
    //regul.VANC_PID(50, 0.3, 0.2, 200);
    //regul.Retract(150, 0.3, 1);
    // 
    // 
    //for (int y_ = 0; y_ < 10; y_++) {
    //    for (int x_ = 0; x_ < 10; x_++) {
    //        regul.TouchScan(0.4, 0.05, 0.05, 0, 0.02, 2, 0, 0.02, 2, MIN_STEP_SIZE, 3, 200);
    //        regul.StepXY(1, 0);
    //    }
    //    regul.StepXY(-10, 0,1,5,5,1000000);
    //    regul.StepXY(0, 1);
    //}
    // 
    // 
    // for (int count = 0; count < 20; count++) {
    //    regul.TouchScan(0.4, 0.15, 0.08, 0, 0.05, 5, 0, 0.025, 0.3, MIN_STEP_SIZE, 2, 200);
    //    //regul.StepXY(1, 0);
    //}

    /*uwait(1000000 * 300);
    for (int count = 0; count < 20; count++) {
        regul.TouchScan(0.4, 0.175, 0.08, 0, 0.05, 5, 0, 0.025, 0.3, MIN_STEP_SIZE, 2, 200);
        regul.StepXY(-1, 0);
    }*/
    //regul.TouchScan(0.4, 0.175, 0.08, 0, 0.05, 5, 0, 0.025, 0.3, MIN_STEP_SIZE, 2, 200);
    







   //                   Перемещения                   //
   //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
   //regul.Retract(10, 0.3, 2);
   //regul.StepXY(-10, 10);
   //regul.ZStep(FORWARD);
   //regul.Landing(2, 5, 0.25);
   // regul.ZCard.BackstepZ();
    //regul.piezo.MoveTo(Vecter(1.0, 1.0, 0), 100, MIN_STEP_SIZE, regul.ZCard, regul.XYCard);
     

 
   //                   Сканы и измерения                    //
   //////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!////////
   //regul.TouchScan(0.3, 0.15, 0.08, 4, 4, 0.01, 0.01, MIN_STEP_SIZE, 2);
   //regul.VANC_PID(20);





    //                   КАЛИБРОВКИ                     //
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
