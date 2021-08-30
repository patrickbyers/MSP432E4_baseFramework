/*
 * baseDevice.h
 *
 *  Created on: Jul 23, 2021
 *      Author: pbyers
 */

#ifndef BASEDEVICE_H_
#define BASEDEVICE_H_

#include <stdint.h>
#include <string>

#define PORT_A  0x40058000
#define PORT_B  0x40059000
#define PORT_C  0x4005A000
#define PORT_D  0x4005B000
#define PORT_E  0x4005C000
#define PORT_F  0x4005D000
#define PORT_G  0x4005E000
#define PORT_H  0x4005F000
//#define PORT_I
#define PORT_J  0x40060000
#define PORT_K  0x40061000
#define PORT_L  0x40062000
#define PORT_M  0x40063000
#define PORT_N  0x40064000
//#define PORT_O
#define PORT_P  0x40065000
#define PORT_Q  0x40066000
#define PORT_R  0x40067000
#define PORT_S  0x40068000
#define PORT_T  0x40069000

#define PIN_0   0x01
#define PIN_1   0x02
#define PIN_2   0x04
#define PIN_3   0x08
#define PIN_4   0x10
#define PIN_5   0x20
#define PIN_6   0x40
#define PIN_7   0x80


class BaseDevice {
    public:
        BaseDevice(const char* name, uint32_t port, uint8_t pin, std::string devName) :
            m_pName(name), m_pDevName(devName), mConfigId(0xFFFF), mPort(port), mPin(pin), mValue(0) {}
//        BaseDevice(char* devName, uint8_t port, uint8_t pin, uint16_t configId);
        BaseDevice(const char* name, uint32_t port, uint8_t pin, std::string devName, uint16_t configId) :
                    m_pName(name), m_pDevName(devName), mConfigId(configId), mPort(port), mPin(pin), mValue(0) {}
        virtual void SetValue(int val) { mValue = val; }
        virtual int GetValue() { return mValue; }
        virtual void SetConfigId(uint16_t id) { mConfigId = id; }
        char GetPort() {
            char port = (char) ((((mPort & 0x000FF000) >> 12) - 0x58) + 'A');
            if (port > 'H') port++;
            if (port > 'O') port++;
            return (port);
        }
        char GetPin() {
            char pin = -1;
            while (++pin < 8) { if((0x01 << pin) & mPin) break; }
            return (pin);
        }

//    protected:
        std::string m_pName;
        std::string m_pDevName;

    protected:
        uint16_t mConfigId;
        uint32_t mPort;
        uint8_t  mPin;
        int      mValue;
};



#endif /* BASEDEVICE_H_ */
