#include "SerialCommunicator.h"

bool SerialCommunicator::getCommand(int16_t &value)
{
    constexpr uint8_t PACKET_SIZE = sizeof(int16_t);  // = 2

    // انتظر وصول بايتَين على الأقل
    if (Serial.available() >= PACKET_SIZE)
    {
        // اقرأ البايتَين مباشرةً في المتغيّر (ليتل إنديَن)
        Serial.readBytes(reinterpret_cast<char*>(&value), PACKET_SIZE);
        return true;
    }
    return false;
}
