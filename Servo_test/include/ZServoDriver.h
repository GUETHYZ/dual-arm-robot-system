/**
 * @file ZServoDriver.h
 * @brief 众灵科技 (Zhongling) ZP Series Bus Servo Driver for ESP32S3
 *
 * Implements the complete Zhongling ASCII protocol over UART.
 * All commands are ASCII text strings terminated by '!' and sent with a line break,
 * matching the vendor ESP32 example style.
 *
 * Protocol format:
 *   Single servo:  #000P1500T1000!    (ID=000, PWM=1500, Time=1000ms)
 *   Multi servo:   {#000P1500T1000!#001P1600T1000!}
 *   Read angle:    #000PRAD!          → response: #000P1500!
 *   Stop:          #000PDST!
 *   Pause:         #000PDPT!
 *   Continue:      #000PDCT!
 *   Torque off:    #000PULK!
 *   Torque on:     #000PULR!
 *
 * Hardware Setup:
 *   ESP32S3 Serial TX → Zhongling driver board → Bus servo
 *   The driver board handles half-duplex bus conversion internally.
 *
 * Reference:
 *   众灵科技 ESP32 example — uses bare Serial.println() to send commands.
 */

#ifndef ZSERVODRIVER_H
#define ZSERVODRIVER_H

#include <Arduino.h>

// ──────────────────────────────────────────────
//  Protocol Constants
// ──────────────────────────────────────────────

/// PWM range for Zhongling ZP series servos
#define ZSERVO_PWM_MIN 500
#define ZSERVO_PWM_MAX 2500
#define ZSERVO_PWM_MID 1500

/// ID range
#define ZSERVO_ID_MIN 0
#define ZSERVO_ID_MAX 254
#define ZSERVO_BROADCAST_ID 255

/// Time range (ms)
#define ZSERVO_TIME_MIN 0
#define ZSERVO_TIME_MAX 9999

/// Command buffer size (large enough for multi-servo commands)
#define ZSERVO_CMD_BUF_SIZE 512

/// Response buffer size
#define ZSERVO_RSP_BUF_SIZE 128

/// Default servo bus baud rate
#define ZSERVO_DEFAULT_BAUD 115200

/// Response timeout (ms)
#define ZSERVO_RESPONSE_TIMEOUT 200

// ──────────────────────────────────────────────
//  Baud Rate Codes (for setBaud)
// ──────────────────────────────────────────────

enum ZServoBaud : uint8_t
{
    ZS_BAUD_9600 = 1,
    ZS_BAUD_19200 = 2,
    ZS_BAUD_38400 = 3,
    ZS_BAUD_57600 = 4,
    ZS_BAUD_115200 = 5,
    ZS_BAUD_128000 = 6,
    ZS_BAUD_256000 = 7,
    ZS_BAUD_1000000 = 8
};

// ──────────────────────────────────────────────
//  Work Mode Values (for setMode)
// ──────────────────────────────────────────────

enum ZServoMode : uint8_t
{
    ZS_MODE_270_CW = 1,        ///< 270° range, clockwise
    ZS_MODE_270_CCW = 2,       ///< 270° range, counter-clockwise
    ZS_MODE_180_CW = 3,        ///< 180° range, clockwise
    ZS_MODE_180_CCW = 4,       ///< 180° range, counter-clockwise
    ZS_MODE_MOTOR_360_CW = 5,  ///< Motor mode, 360° clockwise
    ZS_MODE_MOTOR_360_CCW = 6, ///< Motor mode, 360° counter-clockwise
    ZS_MODE_MOTOR_360_CW2 = 7, ///< Motor mode (alt), 360° clockwise
    ZS_MODE_MOTOR_360_CCW2 = 8 ///< Motor mode (alt), 360° counter-clockwise
};

// ──────────────────────────────────────────────
//  Servo Pose Struct (for multi-servo commands)
// ──────────────────────────────────────────────

struct ServoPose
{
    uint8_t id;
    uint16_t pwm;
    uint16_t timeMs;
};

// ──────────────────────────────────────────────
//  ZServoDriver Class
// ──────────────────────────────────────────────

class ZServoDriver
{
public:
    // ============================================================
    //  Initialization
    // ============================================================

    /**
     * @brief Initialize the servo driver.
     * @param serial   HardwareSerial reference (e.g. Serial1)
     * @param baud     Baud rate (default 115200, must match servo setting)
     * @param dirPin   Direction control pin for RS-485 (-1 = not used)
     */
    void begin(HardwareSerial &serial, unsigned long baud = ZSERVO_DEFAULT_BAUD,
               int8_t dirPin = -1);

    // ============================================================
    //  Single Servo Motion
    // ============================================================

    /**
     * @brief Move a single servo to target PWM position.
     * @param id      Servo ID (0~254)
     * @param pwm     Target PWM (500~2500, 1500 = mid)
     * @param timeMs  Movement time in ms (0~9999)
     * @return true if command was sent
     */
    bool moveServo(uint8_t id, uint16_t pwm, uint16_t timeMs);

    /**
     * @brief Move multiple servos synchronously (wrapped in { }).
     * @param poses  Array of servo poses
     * @param count  Number of servos in the array
     * @return true if command was sent
     */
    bool moveServos(const ServoPose *poses, uint8_t count);

    // ============================================================
    //  Motion Control
    // ============================================================

    /// @brief Immediately stop a servo. Command: #000PDST!
    bool stopServo(uint8_t id);

    /// @brief Pause a servo's current motion. Command: #000PDPT!
    bool pauseServo(uint8_t id);

    /// @brief Resume a paused servo. Command: #000PDCT!
    bool resumeServo(uint8_t id);

    // ============================================================
    //  Torque Control
    // ============================================================

    /// @brief Release torque (free spinning). Command: #000PULK!
    bool torqueOff(uint8_t id);

    /// @brief Restore torque (hold position). Command: #000PULR!
    bool torqueOn(uint8_t id);

    /// @brief Release torque without resistance. Command: #000PULM!
    bool torqueOffNoResist(uint8_t id);

    // ============================================================
    //  Configuration Commands
    // ============================================================

    /**
     * @brief Set servo ID. Command: #000PID001!
     * @param oldId  Current servo ID
     * @param newId  New ID (0~254)
     * @note Servo must be the only one on the bus when changing ID.
     */
    bool setID(uint8_t oldId, uint8_t newId);

    /**
     * @brief Set baud rate. Command: #000PBD5!
     * @param id    Servo ID
     * @param code  Baud rate code (see ZServoBaud enum: 1~8)
     */
    bool setBaud(uint8_t id, uint8_t code);

    /**
     * @brief Set work mode. Command: #000PMOD1!
     * @param id    Servo ID
     * @param mode  Mode value (see ZServoMode enum: 1~8)
     */
    bool setMode(uint8_t id, uint8_t mode);

    /// @brief Factory reset (clears all settings including ID). Command: #000PCLE!
    bool factoryReset(uint8_t id);

    /// @brief Set current position as boot-up default. Command: #000PCSD!
    bool setBootValue(uint8_t id);

    /// @brief Set boot-up torque-off mode. Command: #000PCSM!
    bool setBootTorqueOff(uint8_t id);

    /**
     * @brief Calibrate mid-point offset. Command: #000PSCK+050! or #000PSCK-050!
     * @param id      Servo ID
     * @param offset  Signed offset from 1500 (e.g. +50 or -50)
     */
    bool setMidOffset(uint8_t id, int16_t offset);

    /**
     * @brief Set PID parameters. Command: #000PPAAAIBBB!
     * @param id  Servo ID
     * @param kp  Proportional gain (0~999)
     * @param ki  Integral gain (0~999)
     */
    bool setPID(uint8_t id, uint16_t kp, uint16_t ki);

    /**
     * @brief Set over-temperature protection. Command: #000PSTB=60!
     * @param id    Servo ID
     * @param temp  Temperature threshold in °C (25~80)
     */
    bool setProtectionTemp(uint8_t id, uint8_t temp);

    /// @brief Set minimum position limit to current position. Command: #000PMIN!
    bool setMinLimit(uint8_t id);

    /// @brief Set maximum position limit to current position. Command: #000PMAX!
    bool setMaxLimit(uint8_t id);

    // ============================================================
    //  Query Commands (fire-and-forget — response read separately)
    // ============================================================

    /// @brief Request servo ID. Command: #000PID! Response: #000PID001!
    bool readID(uint8_t id = 0);

    /// @brief Request firmware version. Command: #000PVER!
    bool readVersion(uint8_t id);

    /// @brief Request current work mode. Command: #000PMOD!
    bool readMode(uint8_t id);

    /// @brief Request current angle. Command: #000PRAD! Response: #000P1500!
    bool readAngle(uint8_t id);

    /// @brief Request voltage and temperature. Command: #000PRTV!
    bool readVoltageTemp(uint8_t id);

    /// @brief Request protection temperature. Command: #000PSTB!
    bool readProtection(uint8_t id);

    // ============================================================
    //  LED Control
    // ============================================================

    /// @brief Turn on RGB LED. Command: #000PLN!
    bool ledOn(uint8_t id);

    /// @brief Turn off RGB LED. Command: #000PLF!
    bool ledOff(uint8_t id);

    // ============================================================
    //  Response Parsing (for commands that return data)
    // ============================================================

    /**
     * @brief Wait for and read a response line from the servo bus.
     * @param buffer    Output buffer for response string
     * @param bufSize   Buffer size
     * @param timeoutMs Max wait time in ms
     * @return Length of response, or 0 on timeout, -1 on overflow
     */
    int readResponse(char *buffer, size_t bufSize, uint32_t timeoutMs = ZSERVO_RESPONSE_TIMEOUT);

    /**
     * @brief Parse angle response string: "#000P1500!" → id=0, pwm=1500
     * @param response  The response string (null-terminated)
     * @param outId     Output: parsed servo ID
     * @param outPwm    Output: parsed PWM value
     * @return true if parsing succeeded
     */
    static bool parseAngleResponse(const char *response, uint8_t *outId, uint16_t *outPwm);

    /**
     * @brief Parse ID response string: "#000PID001!" → id=1
     * @param response  The response string
     * @param outId     Output: parsed ID
     * @return true if parsing succeeded
     */
    static bool parseIDResponse(const char *response, uint8_t *outId);

    /**
     * @brief Get the last command string that was sent (for debugging).
     */
    const char *getLastCommand() const { return _cmdBuffer; }

    /**
     * @brief Get direct access to the command buffer.
     */
    char *getCmdBuffer() { return _cmdBuffer; }

private:
    HardwareSerial *_serial = nullptr;
    int8_t _dirPin = -1;
    unsigned long _baud = ZSERVO_DEFAULT_BAUD;

    /// Static command buffer (avoids heap allocation)
    char _cmdBuffer[ZSERVO_CMD_BUF_SIZE];

    // ============================================================
    //  Internal Helpers
    // ============================================================

    /// Validate servo ID
    bool checkId(uint8_t id);

    /// Validate PWM value
    bool checkPwm(uint16_t pwm);

    /// Validate time value
    bool checkTime(uint16_t timeMs);

    /// Send the command string in _cmdBuffer to the servo bus
    bool sendCommand();

    /// Send a raw string to the servo bus
    bool sendRaw(const char *cmd);
};

#endif // ZSERVODRIVER_H
