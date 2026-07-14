/**
 * @file SerialManager.cpp
 * @brief 串口管理器实现
 *
 * 统一管理所有串口的初始化与发送。
 * 每个发送通道使用独立 Mutex，发送后自动 flush()。
 */

#include "comm/SerialManager.h"
#include "config/HardwareConfig.h"
#include "config/TaskConfig.h"

// 静态成员定义
SemaphoreHandle_t SerialManager::groundMutex  = nullptr;
SemaphoreHandle_t SerialManager::jetsonMutex  = nullptr;
SemaphoreHandle_t SerialManager::chassisMutex = nullptr;
SemaphoreHandle_t SerialManager::servoMutex   = nullptr;

void SerialManager::beginAll() {
    DebugSerial.println("[SERIAL] 正在初始化所有串口...");

    // 创建 Mutex
    groundMutex  = xSemaphoreCreateMutex();
    jetsonMutex  = xSemaphoreCreateMutex();
    chassisMutex = xSemaphoreCreateMutex();
    servoMutex   = xSemaphoreCreateMutex();

    if (!groundMutex || !jetsonMutex || !chassisMutex || !servoMutex) {
        DebugSerial.println("[SERIAL] 错误：Mutex 创建失败！");
    }

    // 调试串口：USB CDC，仅用于日志输出，不参与业务通信。
    DebugSerial.begin(DEBUG_BAUD);
    delay(100);
    DebugSerial.println("[SERIAL] 调试串口已启动 (USB CDC，业务串口不复用)");

    // 地面站无线 TTL 串口：物理引脚 RX=GPIO18, TX=GPIO17。
    initSerial(GroundSerial, PIN_GROUND_RX, PIN_GROUND_TX,
               GROUND_BAUD, "地面站/LORA");

    // 舵机控制板串口：物理引脚 RX=GPIO11, TX=GPIO12。
    initSerial(ServoSerial, PIN_SERVO_RX, PIN_SERVO_TX,
               SERVO_BAUD, "舵机控制板");

    // Jetson 与 STM32 单向复用 UART0：RX 接 Jetson，TX 接 STM32。
    initSerial(JetsonSerial, PIN_SHARED_RX, PIN_SHARED_TX,
               JETSON_BAUD, "Jetson/STM32共享串口");
    DebugSerial.println("[SERIAL] STM32底盘与 Jetson 共用 UART0（RX接Jetson，TX接STM32）");

    // 舵机方向控制引脚
    if (PIN_SERVO_DIR >= 0) {
        pinMode(PIN_SERVO_DIR, OUTPUT);
        digitalWrite(PIN_SERVO_DIR, LOW);
        DebugSerial.printf("[SERIAL] 舵机方向引脚 GPIO%d 已配置\n", PIN_SERVO_DIR);
    }

    // LED 引脚
    if (PIN_LED_STATUS >= 0) {
        pinMode(PIN_LED_STATUS, OUTPUT);
        digitalWrite(PIN_LED_STATUS, LOW);
    }

    // 蜂鸣器引脚
    if (PIN_BUZZER >= 0) {
        pinMode(PIN_BUZZER, OUTPUT);
        digitalWrite(PIN_BUZZER, LOW);
    }

    // 急停按钮引脚
    if (PIN_EMERGENCY_STOP >= 0) {
        pinMode(PIN_EMERGENCY_STOP, INPUT_PULLUP);
    }

    DebugSerial.println("[SERIAL] 所有串口初始化完成");
}

void SerialManager::initSerial(HardwareSerial& serial, int rxPin, int txPin,
                               uint32_t baud, const char* name) {
    // Arduino-ESP32 HardwareSerial::setPins(rxPin, txPin)，RX 在前、TX 在后。
    serial.setPins(rxPin, txPin);
    serial.begin(baud, SERIAL_8N1);

    DebugSerial.printf("[SERIAL] %s 串口已启动 (RX=GPIO%d, TX=GPIO%d, Baud=%d)\n",
                       name, rxPin, txPin, baud);
}

// ============================================================
// 二进制发送（底层通用函数）
// ============================================================

bool SerialManager::sendBinary(Stream& stream, SemaphoreHandle_t mutex,
                               const uint8_t* data, size_t len,
                               const char* name) {
    if (data == nullptr || len == 0) return false;
    if (mutex == nullptr) return false;

    // 获取 Mutex（带超时）
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(TX_MUTEX_TIMEOUT)) != pdTRUE) {
        DebugSerial.printf("[SERIAL] 错误：%s 发送 Mutex 超时\n", name);
        return false;
    }

    size_t written = stream.write(data, len);
    stream.flush();

    xSemaphoreGive(mutex);

    if (written != len) {
        DebugSerial.printf("[SERIAL] 错误：%s 发送不完整 (%d/%d 字节)\n",
                          name, (int)written, (int)len);
        return false;
    }

    return true;
}

// ============================================================
// 公开发送接口
// ============================================================

bool SerialManager::sendToGround(const uint8_t* data, size_t len) {
    return sendBinary(GroundSerial, groundMutex, data, len, "地面站");
}

bool SerialManager::sendToJetson(const uint8_t* data, size_t len) {
    DebugSerial.println("[SERIAL] 当前拓扑下 Jetson 仅接收输入，不支持向 Jetson 发送");
    return false;
}

bool SerialManager::sendToChassis(const uint8_t* data, size_t len) {
    return sendBinary(ChassisSerial, chassisMutex, data, len, "STM32底盘");
}

bool SerialManager::sendToServo(const char* cmd) {
    if (cmd == nullptr || cmd[0] == '\0') return false;
    if (servoMutex == nullptr) return false;

    size_t len = strlen(cmd);

    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(TX_MUTEX_TIMEOUT)) != pdTRUE) {
        DebugSerial.println("[SERIAL] 错误：舵机发送 Mutex 超时");
        return false;
    }

    // 与 Servo_test/ZServoDriver 保持一致：ASCII 命令 + println() 行尾。
    size_t written = ServoSerial.print(cmd);
    written += ServoSerial.println();
    ServoSerial.flush();

    xSemaphoreGive(servoMutex);

    const size_t expected = len + 2;
    if (written != expected) {
        DebugSerial.printf("[SERIAL] 错误：舵机发送不完整 (%d/%d 字节)\n",
                          (int)written, (int)expected);
        return false;
    }

    return true;
}

// ============================================================
// 协议帧发送
// ============================================================

bool SerialManager::sendFrame(const ProtocolFrame& frame, char target) {
    uint8_t buffer[PROTOCOL_FRAME_MAX];
    size_t len = protocolPack(frame, buffer, sizeof(buffer));

    if (len == 0) {
        DebugSerial.println("[SERIAL] 错误：协议帧打包失败");
        return false;
    }

    switch (target) {
    case 'G':
        return sendToGround(buffer, len);
    case 'J':
        return sendToJetson(buffer, len);
    case 'C':
        return sendToChassis(buffer, len);
    default:
        DebugSerial.printf("[SERIAL] 错误：未知发送目标 '%c'\n", target);
        return false;
    }
}
