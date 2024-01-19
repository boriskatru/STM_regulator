#pragma once


#include <iostream>
#include <ziAPI.h>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <map>
#include <string>
#include "ziUtils.hpp"
#include "Settings.h"

using namespace std;


class ZurichMFLI {
    ZIConnection conn = nullptr;
    const char* dataServer;
    const uint16_t port = 8004;
    const char* deviceInterface;
    char nodePath[1024];
    int filterOrder = 3;

    ZIDoubleData tm_const_ffreq(ZIDoubleData freq);
public:

    const char* deviceAddress;
    ZurichMFLI(const char* dev = "dev4569"); 

    void setOscFreq(ZIDoubleData freq);

    void setDCAmplitude(ZIDoubleData offset);

    void setACAmplitude(ZIDoubleData amplitude);

    void setSincEnable();

    void setSincDisable();

    void setDemodFilterFreq(ZIDoubleData freq);

    void setDemodFilterOrder(ZIIntegerData order);

    void setSignalEnable();

    void setSignalDisable();

    void setACEnable();

    void setACDisable();

    void setDemodRate(uint32_t index, ZIDoubleData rate);

    ZIDoubleData getDemodRate(uint32_t index);

    ~ZurichMFLI();


};


