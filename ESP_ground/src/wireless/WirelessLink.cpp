/**
 * @file WirelessLink.cpp
 * @brief LoRa 无线串口通信模块实现
 */

#include "wireless/WirelessLink.h"
#include "config/TaskConfig.h"

// ============================================================================
// 静态成员初始化
// ============================================================================

ProtocolParser WirelessLink::parser;
bool WirelessLink::initialized = false;

// ============================================================================
// 初始化
// ============================================================================

void WirelessLink::begin() {
    Lora.setPins(LORA_RX_PIN, LORA_TX_PIN);
    Lora.begin(LORA_UART_BAUD);

    // 创建发送互斥锁
    if (loraTxMutex == nullptr) {
        loraTxMutex = xSemaphoreCreateMutex();
    }

    initialized = true;

    DebugSerial.println(F("[LORA] wireless link initialized"));
    DebugSerial.printf("[LORA] RX_PIN=%d, TX_PIN=%d, BAUD=%lu\n",
                       LORA_RX_PIN, LORA_TX_PIN, LORA_UART_BAUD);
}

// ============================================================================
// 发送
// ============================================================================

bool WirelessLink::sendFrame(const ProtocolFrame &frame) {
    if (!initialized) {
        DebugSerial.println(F("[LORA TX] ERROR: not initialized"));
        return false;
    }

    // 打包协议帧
    uint8_t txBuf[FRAME_MAX_LEN];
    size_t txLen = 0;

    if (!packFrame(frame, txBuf, sizeof(txBuf), txLen)) {
        DebugSerial.println(F("[LORA TX] ERROR: pack failed"));
        return false;
    }

    // 获取互斥锁后发送
    if (xSemaphoreTake(loraTxMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        size_t written = Lora.write(txBuf, txLen);
        Lora.flush();
        xSemaphoreGive(loraTxMutex);

        if (written == txLen) {
            DebugSerial.printf("[LORA TX] %s (seq=%u, len=%u)\n",
                               funcCodeToString(frame.funcCode),
                               frame.seqNum, txLen);
            return true;
        } else {
            DebugSerial.printf("[LORA TX] ERROR: write mismatch %u/%u\n",
                               written, txLen);
            return false;
        }
    } else {
        DebugSerial.println(F("[LORA TX] ERROR: mutex timeout"));
        return false;
    }
}

// ============================================================================
// 接收
// ============================================================================

bool WirelessLink::pollFrame(ProtocolFrame &frame) {
    if (!initialized) {
        return false;
    }

    size_t processed = 0;

    while (Lora.available() > 0 && processed < LORA_RX_MAX_BYTES_PER_CYCLE) {
        uint8_t byte = static_cast<uint8_t>(Lora.read());
        processed++;

        if (parser.inputByte(byte, frame)) {
            // 解析出一个完整帧
            return true;
        }
    }

    return false;
}
