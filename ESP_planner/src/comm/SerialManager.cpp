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

    // 调试串口（USB CDC，通常无需额外配置）
    // 如果 DebugSerial 和 GroundSerial 都是 USB Serial，只需 begin 一次
    DebugSerial.begin(DEBUG_BAUD);
    // 等待 USB CDC 连接稳定
    delay(100);
    DebugSerial.println("[SERIAL] 调试串口已启动 (USB CDC)");

    // 地面站串口
    if (&GroundSerial != &DebugSerial) {
        // 地面站使用独立串口
        initSerial(GroundSerial, PIN_GROUND_TX, PIN_GROUND_RX,
                   GROUND_BAUD, "地面站");
    } else {
        DebugSerial.println("[SERIAL] 地面站串口 = 调试串口 (USB CDC 复用)");
    }

    // Jetson 串口
    if (&JetsonSerial != &GroundSerial && &JetsonSerial != &DebugSerial) {
        initSerial(JetsonSerial, PIN_JETSON_TX, PIN_JETSON_RX,
                   JETSON_BAUD, "Jetson");
    } else {
        DebugSerial.println("[SERIAL] Jetson 串口与调试/地面站串口共享");
    }

    // 底盘串口
    if (&ChassisSerial != &JetsonSerial && &ChassisSerial != &GroundSerial
        && &ChassisSerial != &DebugSerial) {
        initSerial(ChassisSerial, PIN_CHASSIS_TX, PIN_CHASSIS_RX,
                   CHASSIS_BAUD, "STM32底盘");
    } else {
        DebugSerial.println("[SERIAL] 警告：底盘串口与其他串口共享硬件！");
        DebugSerial.println("[SERIAL] 请检查 HardwareConfig.h 中的串口配置");
    }

    // 舵机串口
    if (&ServoSerial != &ChassisSerial && &ServoSerial != &JetsonSerial
        && &ServoSerial != &GroundSerial && &ServoSerial != &DebugSerial) {
        initSerial(ServoSerial, PIN_SERVO_TX, PIN_SERVO_RX,
                   SERVO_BAUD, "舵机控制板");
    } else {
        DebugSerial.println("[SERIAL] 警告：舵机串口与其他串口共享硬件！");
        DebugSerial.println("[SERIAL] 请检查 HardwareConfig.h 中的串口配置");
    }

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

void SerialManager::initSerial(HardwareSerial& serial, int txPin, int rxPin,
                               uint32_t baud, const char* name) {
    // 注意：setPins() 参数顺序可能因 Arduino Core 版本不同而异
    // 本工程默认使用 setPins(TX, RX) 顺序
    // 如果串口无数据，请尝试交换为 setPins(RX, TX)
    serial.setPins(txPin, rxPin);
    serial.begin(baud);

    DebugSerial.printf("[SERIAL] %s 串口已启动 (TX=GPIO%d, RX=GPIO%d, Baud=%d)\n",
                       name, txPin, rxPin, baud);
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
    return sendBinary(JetsonSerial, jetsonMutex, data, len, "Jetson");
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

    size_t written = ServoSerial.write((const uint8_t*)cmd, len);
    ServoSerial.flush();

    xSemaphoreGive(servoMutex);

    if (written != len) {
        DebugSerial.printf("[SERIAL] 错误：舵机发送不完整 (%d/%d 字节)\n",
                          (int)written, (int)len);
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
