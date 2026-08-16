#include "defaultGateway.h"
#include <arpa/inet.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

std::string getDefaultGateway()
{
    return "100.80.80.15";
    /*
    std::ifstream file("/proc/net/route");
    if (!file.is_open())
    {
        return "";
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);

        std::string iface, destination, gatewayHex;
        unsigned flags, refcnt, use, metric, mask, mtu, win, irtt;

        if (!(iss >> iface >> destination >> gatewayHex >> flags >> refcnt >> use >> metric >> mask >> mtu >> win >>
              irtt))
        {
            continue;
        }

        // Destination 00000000 = default route
        if (destination == "00000000")
        {
            unsigned long gatewayUL = std::stoul(gatewayHex, nullptr, 16);

            struct in_addr addr;
            addr.s_addr = static_cast<uint32_t>(gatewayUL);

            return inet_ntoa(addr);
        }
    }

    return "";
    */
}
