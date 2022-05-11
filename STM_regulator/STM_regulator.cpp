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
using namespace std::chrono;
int main()
{

    
    

    setlocale(LC_ALL, "Russian");
    
    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentProcess(), THREAD_PRIORITY_TIME_CRITICAL);
    
    cout << GetPriorityClass(GetCurrentProcess()) << endl;
   // ofstream file;
   // file.open("VAC.txt" , std::ofstream::out);
   Regulator regul;

   ADC_Collect data = regul.XYCard.AnalogRead();
   for (int i = 0; i < 3; i++) {
       cout << "ch " << i << " value: " << data.Average(8, i) << endl;
   }

   //regul.Retract(3);
   //regul.VANC_(3, -3, 0.003,1);
  // getchar(); getchar();
   Timer tmr;
   tmr.set_to_zero();
   cout << "JUMP STARTED" << endl;
   while (tmr.get_full_interval() < 500000000)
   {
       regul.piezo.ZFJumpTo(1 + 1 * sin(tmr.get_full_interval() / 10000), regul.ZCard);
   }
   cout << "JUMP ENDED" << endl;
   //regul.Landing(3,5, -0.015);
   //regul.ClearTip(100);
   
   

   //regul.StepXY(0, -3);
   //getchar(); getchar();
   //regul.Landing(3, 5 , -0.01);


   regul.rise(1,9000000);
   cout << "PID started" << endl;
   //int volt = 0.55;
   regul.IntPID_exp(1, -2, 150000000);
   int cnt_run = 5;
   int cnt_seq = 20;
   for (int i = 0; i < cnt_seq; i++) {
       cout << "PID" << endl;
       regul.IntPID_exp(1, -2, 5000000);
       cout << "graph number: " << i << endl << endl;
       for (int k = 0; k < cnt_run; k++) {
           regul.VANC_(2, -2, 0.002, i * cnt_run + k);
       }

       cout << "done!" << endl;
       //cout << "rise started" << endl;
      // regul.rise(3, 1000000);
       /*cout << " input v";
       cin >> volt;*/
       
   }
  

   //regul.TouchScan(1,0.2,-0.01,5,5,0.05,0.05);

   regul.~Regulator();
   //regul.Landing();














    //LCard Card1(1), Card2(2);
    //cout << Card1.hnd << /*endl << Card2.hnd <<*/ endl;
  
    //
    //double iters = 100000;
    //auto start = std::chrono::high_resolution_clock::now();
   
    //for (int i = 0; i < iters; i++) {
    //   // Card2.AsyncSingleAnalogRead(0);
    //    Card2.AnalogRead();
    //    if (i % 500 == 0) {
    //       // cout << "Count= "<<Card2.count_ADC_data << endl;
    //        cout << Card2.current_data[0] << endl;
    //    }
    //   // Card2.SingleAnalogOut(1+0.001*sin(chrono::duration<double, std::micro>(std::chrono::high_resolution_clock::now() -start).count()*2*3.141593/10000));
    //}
    //auto end = std::chrono::high_resolution_clock::now();
    //double ttime = chrono::duration<double, std::micro>(end - start).count();
    //cout << "total time, ms: " << ttime / 1000 << endl;
    //cout << "average DAC frequency,MHz:" << iters/ttime<< endl;
    //int err = L502_StreamsStop(Card1.hnd);
    //err = L502_StreamsStop(Card2.hnd);
    //if (err != 0) cerr << "Ошибка  " << err << " в L502_StreamsStart()" << endl;
    //Card1.~LCard(); Card2.~LCard();

   
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
