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


using namespace std;
void runtime_msg(ZIConnection conn, std::runtime_error& e) {
    char extErrorMessage[1024] = "";
    ziAPIGetLastError(conn, extErrorMessage, 1024);
    fprintf(stderr, "[ERROR] %s\ndetails: `%s`\n.", e.what(), extErrorMessage);
    cerr << "No Zurich MFLI CONNECTION";

}

class ZurichMFLI {
    ZIConnection conn = nullptr;
    const char* dataServer;
    const uint16_t port = 8004;
    const char* deviceInterface;
    char nodePath[1024];
    int filterOrder = 3;

    ZIDoubleData tm_const_ffreq(ZIDoubleData freq) {
        return (0.2435 / (double)freq / (double)filterOrder);
    }
public:

    const char* deviceAddress;
    ZurichMFLI(const char* dev = "dev4569") {
        deviceAddress = ziUtilsGetEnv("LABONE_DEVICE", dev);
        printf("ENV LABONE_DEVICE=%s\n", deviceAddress);
        dataServer = ziUtilsGetEnv("LABONE_SERVER", "192.168.1.172");
        deviceInterface = "PCIe";

        if (isError(ziAPIInit(&conn)))
        {
            cerr << "Nj Zurich " << dev << "   MFLI CONNECTION";
        }

        ziAPISetDebugLevel(0);
        ziAPIWriteDebugLog(0, "Logging enabled.");
        try {
            checkError(ziAPIConnectEx(conn, dataServer, port, ZI_API_VERSION_6, nullptr));
            ziApiServerVersionCheck(conn);
            checkError(ziAPIConnectDevice(conn, deviceAddress, deviceInterface, nullptr));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
        setACEnable();
        setSignalEnable();
    }

    void setOscFreq(ZIDoubleData freq) {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/oscs/0/freq", deviceAddress);
            checkError(ziAPISetValueD(conn, nodePath, freq));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setDCAmplitude(ZIDoubleData offset) {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/sigouts/0/offset", deviceAddress);
            checkError(ziAPISetValueD(conn, nodePath, offset));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setACAmplitude(ZIDoubleData amplitude) {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/sigouts/0/amplitudes/1", deviceAddress);
            checkError(ziAPISetValueD(conn, nodePath, amplitude));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setSincEnable() {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/demods/0/sinc", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, 1));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setSincDisable() {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/demods/0/sinc", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, 0));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setDemodFilterFreq(ZIDoubleData freq) {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/demods/0/timeconstant", deviceAddress);
            checkError(ziAPISetValueD(conn, nodePath, tm_const_ffreq(freq)));

        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setDemodFilterOrder(ZIIntegerData order) {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/demods/0/order", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, order));
            filterOrder = order;
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setSignalEnable() {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/sigouts/0/on", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, 1));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setSignalDisable() {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/sigouts/0/on", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, 0));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setACEnable() {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/sigouts/0/enables/1", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, 1));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setACDisable() {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/sigouts/0/enables/1", deviceAddress);
            checkError(ziAPISetValueI(conn, nodePath, 0));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    void setDemodRate(uint32_t index, ZIDoubleData rate) {
        try {
            snprintf(nodePath, sizeof(nodePath), "/%s/demods/%d/rate", deviceAddress, index);
            checkError(ziAPISetValueD(conn, nodePath, rate));
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    ZIDoubleData getDemodRate(uint32_t index)
    {
        try {
            ZIDoubleData rate;

            snprintf(nodePath, sizeof(nodePath), "/%s/demods/%d/rate", deviceAddress, index);
            checkError(ziAPIGetValueD(conn, nodePath, &rate));
            return rate;
        }
        catch (std::runtime_error& e)
        {
            runtime_msg(conn, e);
        }
        catch (...)
        {
            fprintf(stderr, "[ERROR] Unexpected error\n.No Zurich MFLI CONNECTION");
        }
    }

    ~ZurichMFLI() {
        ziAPIDisconnect(conn);
        ziAPIDestroy(conn);
    }


};


