/**
 * @file ZServoDriver.cpp
 * @brief 众灵科技 (Zhongling) ZP Series Bus Servo Driver Implementation
 *
 * All commands are ASCII text strings sent over UART.
 * The Zhongling driver board handles half-duplex bus conversion —
 * the ESP32 simply sends ASCII strings and optionally reads responses.
 */

#include "ZServoDriver.h"
#include <stdio.h>

// ──────────────────────────────────────────────
//  Initialization
// ──────────────────────────────────────────────

void ZServoDriver::begin(HardwareSerial &serial, unsigned long baud, int8_t dirPin)
{
    _serial = &serial;
    _baud = baud;
    _dirPin = dirPin;

    // Configure direction pin if used
    if (_dirPin >= 0)
    {
        pinMode(_dirPin, OUTPUT);
        digitalWrite(_dirPin, LOW); // Default: receive mode
    }

    // Initialize the servo serial port
    _serial->begin(baud, SERIAL_8N1);

    memset(_cmdBuffer, 0, sizeof(_cmdBuffer));
}

// ──────────────────────────────────────────────
//  Parameter Validation
// ──────────────────────────────────────────────

bool ZServoDriver::checkId(uint8_t id)
{
    if (id > ZSERVO_ID_MAX && id != ZSERVO_BROADCAST_ID)
    {
        return false;
    }
    return true;
}

bool ZServoDriver::checkPwm(uint16_t pwm)
{
    return (pwm >= ZSERVO_PWM_MIN && pwm <= ZSERVO_PWM_MAX);
}

bool ZServoDriver::checkTime(uint16_t timeMs)
{
    return (timeMs <= ZSERVO_TIME_MAX);
}

// ──────────────────────────────────────────────
//  Internal: Send Command
// ──────────────────────────────────────────────

bool ZServoDriver::sendCommand()
{
    if (!_serial)
        return false;

    // Set direction to transmit
    if (_dirPin >= 0)
    {
        digitalWrite(_dirPin, HIGH);
        delayMicroseconds(50);
    }

    // Match the vendor example's command style: ASCII command + line break.
    // The servo protocol is terminated by '!' and many boards also accept a newline.
    _serial->print(_cmdBuffer);
    _serial->println();

    // Wait for transmission to complete
    _serial->flush();

    // Set direction back to receive
    if (_dirPin >= 0)
    {
        delayMicroseconds(50);
        digitalWrite(_dirPin, LOW);
    }

    return true;
}

bool ZServoDriver::sendRaw(const char *cmd)
{
    if (!cmd || !_serial)
        return false;

    strncpy(_cmdBuffer, cmd, ZSERVO_CMD_BUF_SIZE - 1);
    _cmdBuffer[ZSERVO_CMD_BUF_SIZE - 1] = '\0';

    return sendCommand();
}

// ──────────────────────────────────────────────
//  Single Servo Motion
// ──────────────────────────────────────────────

bool ZServoDriver::moveServo(uint8_t id, uint16_t pwm, uint16_t timeMs)
{
    if (!checkId(id) || !checkPwm(pwm) || !checkTime(timeMs))
    {
        return false;
    }

    // Format: #000P1500T1000!
    int len = snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE,
                       "#%03dP%04dT%04d!", id, pwm, timeMs);
    if (len < 0 || len >= ZSERVO_CMD_BUF_SIZE)
    {
        return false;
    }

    return sendCommand();
}

// ──────────────────────────────────────────────
//  Multi-Servo Motion
// ──────────────────────────────────────────────

bool ZServoDriver::moveServos(const ServoPose *poses, uint8_t count)
{
    if (!poses || count == 0)
        return false;

    // Validate all parameters first
    for (uint8_t i = 0; i < count; i++)
    {
        if (!checkId(poses[i].id) || !checkPwm(poses[i].pwm) || !checkTime(poses[i].timeMs))
        {
            return false;
        }
    }

    // Build multi-servo command: {#000P1500T1000!#001P1600T1000!}
    size_t pos = 0;
    _cmdBuffer[pos++] = '{';

    for (uint8_t i = 0; i < count; i++)
    {
        int written = snprintf(_cmdBuffer + pos, ZSERVO_CMD_BUF_SIZE - pos,
                               "#%03dP%04dT%04d!",
                               poses[i].id, poses[i].pwm, poses[i].timeMs);
        if (written < 0 || (size_t)written >= ZSERVO_CMD_BUF_SIZE - pos)
        {
            return false; // Buffer overflow
        }
        pos += written;
    }

    if (pos >= ZSERVO_CMD_BUF_SIZE - 1)
        return false;
    _cmdBuffer[pos++] = '}';
    _cmdBuffer[pos] = '\0';

    return sendCommand();
}

// ──────────────────────────────────────────────
//  Motion Control Commands
// ──────────────────────────────────────────────

bool ZServoDriver::stopServo(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPDST!", id);
    return sendCommand();
}

bool ZServoDriver::pauseServo(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPDPT!", id);
    return sendCommand();
}

bool ZServoDriver::resumeServo(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPDCT!", id);
    return sendCommand();
}

// ──────────────────────────────────────────────
//  Torque Control
// ──────────────────────────────────────────────

bool ZServoDriver::torqueOff(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPULK!", id);
    return sendCommand();
}

bool ZServoDriver::torqueOn(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPULR!", id);
    return sendCommand();
}

bool ZServoDriver::torqueOffNoResist(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPULM!", id);
    return sendCommand();
}

// ──────────────────────────────────────────────
//  Configuration Commands
// ──────────────────────────────────────────────

bool ZServoDriver::setID(uint8_t oldId, uint8_t newId)
{
    if (!checkId(oldId) || !checkId(newId))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPID%03d!", oldId, newId);
    return sendCommand();
}

bool ZServoDriver::setBaud(uint8_t id, uint8_t code)
{
    if (!checkId(id) || code < 1 || code > 8)
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPBD%d!", id, code);
    return sendCommand();
}

bool ZServoDriver::setMode(uint8_t id, uint8_t mode)
{
    if (!checkId(id) || mode < 1 || mode > 8)
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPMOD%d!", id, mode);
    return sendCommand();
}

bool ZServoDriver::factoryReset(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPCLE!", id);
    return sendCommand();
}

bool ZServoDriver::setBootValue(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPCSD!", id);
    return sendCommand();
}

bool ZServoDriver::setBootTorqueOff(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPCSM!", id);
    return sendCommand();
}

bool ZServoDriver::setMidOffset(uint8_t id, int16_t offset)
{
    if (!checkId(id))
        return false;
    if (offset >= 0)
    {
        snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPSCK+%03d!", id, offset);
    }
    else
    {
        snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPSCK-%03d!", id, -offset);
    }
    return sendCommand();
}

bool ZServoDriver::setPID(uint8_t id, uint16_t kp, uint16_t ki)
{
    if (!checkId(id) || kp > 999 || ki > 999)
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPP%03dI%03d!", id, kp, ki);
    return sendCommand();
}

bool ZServoDriver::setProtectionTemp(uint8_t id, uint8_t temp)
{
    if (!checkId(id) || temp < 25 || temp > 80)
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPSTB=%d!", id, temp);
    return sendCommand();
}

bool ZServoDriver::setMinLimit(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPMIN!", id);
    return sendCommand();
}

bool ZServoDriver::setMaxLimit(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPMAX!", id);
    return sendCommand();
}

// ──────────────────────────────────────────────
//  Query Commands
// ──────────────────────────────────────────────

bool ZServoDriver::readID(uint8_t id)
{
    // Note: #000PID! uses the current ID (often 000 when unknown)
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPID!", id);
    return sendCommand();
}

bool ZServoDriver::readVersion(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPVER!", id);
    return sendCommand();
}

bool ZServoDriver::readMode(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPMOD!", id);
    return sendCommand();
}

bool ZServoDriver::readAngle(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPRAD!", id);
    return sendCommand();
}

bool ZServoDriver::readVoltageTemp(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPRTV!", id);
    return sendCommand();
}

bool ZServoDriver::readProtection(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPSTB!", id);
    return sendCommand();
}

// ──────────────────────────────────────────────
//  LED Control
// ──────────────────────────────────────────────

bool ZServoDriver::ledOn(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPLN!", id);
    return sendCommand();
}

bool ZServoDriver::ledOff(uint8_t id)
{
    if (!checkId(id))
        return false;
    snprintf(_cmdBuffer, ZSERVO_CMD_BUF_SIZE, "#%03dPLF!", id);
    return sendCommand();
}

// ──────────────────────────────────────────────
//  Response Reading & Parsing
// ──────────────────────────────────────────────

int ZServoDriver::readResponse(char *buffer, size_t bufSize, uint32_t timeoutMs)
{
    if (!_serial || !buffer || bufSize == 0)
        return 0;

    // Set direction to receive
    if (_dirPin >= 0)
    {
        digitalWrite(_dirPin, LOW);
    }

    memset(buffer, 0, bufSize);
    size_t idx = 0;
    uint32_t start = millis();

    while (millis() - start < timeoutMs)
    {
        while (_serial->available() && idx < bufSize - 1)
        {
            char c = (char)_serial->read();
            buffer[idx++] = c;

            // Response ends with '!'
            if (c == '!')
            {
                buffer[idx] = '\0';
                return idx;
            }
        }

        if (idx >= bufSize - 1)
        {
            buffer[bufSize - 1] = '\0';
            return -1; // Buffer overflow
        }

        delay(1); // Yield to other tasks
    }

    buffer[idx] = '\0';
    return (idx > 0) ? idx : 0; // 0 = timeout with no data
}

// ──────────────────────────────────────────────
//  Static Response Parsers
// ──────────────────────────────────────────────

bool ZServoDriver::parseAngleResponse(const char *response, uint8_t *outId, uint16_t *outPwm)
{
    // Expected format: #000P1500!
    //                   0123456789
    if (!response || !outId || !outPwm)
        return false;
    if (strlen(response) < 10)
        return false;
    if (response[0] != '#' || response[4] != 'P' || response[9] != '!')
        return false;

    // Parse ID (positions 1-3)
    int id = (response[1] - '0') * 100 + (response[2] - '0') * 10 + (response[3] - '0');
    if (id < 0 || id > 255)
        return false;

    // Parse PWM (positions 5-8)
    int pwm = (response[5] - '0') * 1000 + (response[6] - '0') * 100 + (response[7] - '0') * 10 + (response[8] - '0');
    if (pwm < 0 || pwm > 9999)
        return false;

    *outId = (uint8_t)id;
    *outPwm = (uint16_t)pwm;
    return true;
}

bool ZServoDriver::parseIDResponse(const char *response, uint8_t *outId)
{
    // Expected format: #000PID001!
    //                   0123456789
    if (!response || !outId)
        return false;
    if (strlen(response) < 10)
        return false;
    if (response[0] != '#' || response[4] != 'P' || response[5] != 'I' || response[6] != 'D')
        return false;

    // Parse returned ID (positions 7-9)
    int id = (response[7] - '0') * 100 + (response[8] - '0') * 10 + (response[9] - '0');
    if (id < 0 || id > 255)
        return false;

    *outId = (uint8_t)id;
    return true;
}
