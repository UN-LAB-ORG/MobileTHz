#include "IMU.h"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    IMU imu;
    std::cout << "Scanning for devices..." << std::endl;
    auto devices = imu.scanForDevices();

    if (devices.empty())
    {
        std::cout << "No devices found." << std::endl;
        return 0;
    }

    std::cout << "\nSelect a device by entering its number:" << std::endl;
    for (size_t i = 0; i < devices.size(); ++i)
    {
        if (!devices[i].name.empty())
        {
            std::cout << (i + 1) << ". " << devices[i].name << std::endl;
        }
    }

    size_t choice;
    std::cin >> choice;

    if (choice > 0 && choice <= devices.size())
    {
        if (imu.connectToDevice(devices[choice - 1]))
        {
            imu.readIMUData(); // Read data
            std::this_thread::sleep_for(std::chrono::seconds(30));
            imu.disconnect();
        }
    }
    else
    {
        std::cout << "Invalid selection." << std::endl;
    }

    return 0;
}