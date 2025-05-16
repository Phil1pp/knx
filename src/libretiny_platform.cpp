#include "libretiny_platform.h"

#ifdef LIBRETINY
#include <Arduino.h>
#include "knx/bits.h"

#include <lwip/netif.h>

#ifndef KNX_SERIAL
#define KNX_SERIAL Serial
#endif

#ifndef KNX_FLASH_OFFSET
#error "KNX_FLASH_OFFSET is not defined. E.g. 0x1DB000 for BK7231N"
#elif (KNX_FLASH_OFFSET % 4096) != 0
#error "KNX_FLASH_OFFSET must be a multiple of 4096"
#endif

static uint8_t NVS_buffer[KNX_FLASH_SIZE];

LibretinyPlatform::LibretinyPlatform()
#ifndef KNX_NO_DEFAULT_UART
    : ArduinoPlatform(&KNX_SERIAL)
#endif
{
    // _memoryType = Flash;
}

LibretinyPlatform::LibretinyPlatform( HardwareSerial* s) : ArduinoPlatform(s)
{
    // _memoryType = Flash;
}

uint32_t LibretinyPlatform::currentIpAddress()
{
    return WiFi.localIP();
}

uint32_t LibretinyPlatform::currentSubnetMask()
{
    return WiFi.subnetMask();
}

uint32_t LibretinyPlatform::currentDefaultGateway()
{
    return WiFi.gatewayIP();
}

void LibretinyPlatform::macAddress(uint8_t * addr)
{
    macAddress(addr);
}

uint32_t LibretinyPlatform::uniqueSerialNumber()
{
    return lt_cpu_get_mac_id();
}

void LibretinyPlatform::restart()
{
    println("restart");
    lt_reboot();
}

void LibretinyPlatform::setupMultiCast(uint32_t addr, uint16_t port)
{
    //workaround for libretiny bug: NETIF_FLAG_IGMP is not set by default
    struct netif *netif;
	for (netif = netif_list; netif != NULL; netif = netif->next) {
		netif->flags |= NETIF_FLAG_IGMP;
	}

    IPAddress mcastaddr(htonl(addr));    
    KNX_DEBUG_SERIAL.printf("setup multicast addr: %d.%d.%d.%d port: %d ip: %d.%d.%d.%d\n", mcastaddr[0], mcastaddr[1], mcastaddr[2], mcastaddr[3], port, WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    uint8_t result = _udp.beginMulticast(mcastaddr, port);
    KNX_DEBUG_SERIAL.printf("multicast setup result %d\n", result);
}

void LibretinyPlatform::closeMultiCast()
{
    _udp.stop();
}

bool LibretinyPlatform::sendBytesMultiCast(uint8_t * buffer, uint16_t len)
{
    _udp.beginMulticastPacket();
    _udp.write(buffer, len);
    _udp.endPacket();
    return true;
}

int LibretinyPlatform::readBytesMultiCast(uint8_t * buffer, uint16_t maxLen)
{
    int len = _udp.parsePacket();
    if (len == 0)
        return 0;
    
    if (len > maxLen)
    {
        KNX_DEBUG_SERIAL.printf("udp buffer to small. was %d, needed %d\n", maxLen, len);
        fatalError();
    }

    _udp.read(buffer, len);
    return len;
}

bool LibretinyPlatform::sendBytesUniCast(uint32_t addr, uint16_t port, uint8_t* buffer, uint16_t len)
{
    IPAddress ucastaddr(htonl(addr));
    println("sendBytesUniCast endPacket fail");
    if(_udp.beginPacket(ucastaddr, port) == 1) {
        _udp.write(buffer, len);
        if(_udp.endPacket() == 0) println("sendBytesUniCast endPacket fail");
    }
    else println("sendBytesUniCast beginPacket fail");
    return true;
}

uint8_t* LibretinyPlatform::getEepromBuffer(uint32_t size)
{
    if (size > KNX_FLASH_SIZE)
    {
        fatalError();
    }

    lt_flash_read(KNX_FLASH_OFFSET, NVS_buffer, KNX_FLASH_SIZE);

    for (int i = 0; i < size; i++)
    {
        if (NVS_buffer[i] != 0)
        {
            return NVS_buffer;
        }
    }

    memset(NVS_buffer, 0xff, size);

    return NVS_buffer;
}

void LibretinyPlatform::commitToEeprom()
{
    println("LibretinyPlatform::commitToEeprom() ...");

    uint32_t result = -1;//lt_flash_write(KNX_FLASH_OFFSET, NVS_buffer, KNX_FLASH_SIZE);

    if (result < 0)
    {
        print("Error writing to FLASH, result: ");
        println(result);
    }
    else
    {
        println("FLASH successfully written, result: ");
        println(result);
    }

    delay(500);
}

#endif
