/**
 * @file TjcPanel.cpp
 * @brief 陶晶驰 TJC 串口屏驱动实现
 */

#include "tjc/TjcPanel.h"

// ============================================================================
// 静态成员初始化
// ============================================================================

TjcPanel::ParseState TjcPanel::parseState = TjcPanel::ParseState::WAIT_HEAD;
uint8_t TjcPanel::cmdByte1 = 0;
uint32_t TjcPanel::discardCount = 0;

// ============================================================================
// 初始化
// ============================================================================

void TjcPanel::begin() {
    TJC.setPins(TJC_RX_PIN, TJC_TX_PIN);
    TJC.begin(TJC_UART_BAUD);

    DebugSerial.println(F("[TJC] panel initialized"));
    DebugSerial.printf("[TJC] RX_PIN=%d, TX_PIN=%d, BAUD=%lu\n",
                       TJC_RX_PIN, TJC_TX_PIN, TJC_UART_BAUD);

    // 清空状态
    parseState = ParseState::WAIT_HEAD;
    cmdByte1 = 0;
    discardCount = 0;
}

// ============================================================================
// 按钮事件轮询（逐字节状态机，非阻塞）
// ============================================================================

bool TjcPanel::pollButtonEvent(uint8_t &cmd) {
    // 循环读取所有可用字节，每轮最多处理一个帧
    while (TJC.available() > 0) {
        uint8_t byte = static_cast<uint8_t>(TJC.read());

        switch (parseState) {

            case ParseState::WAIT_HEAD:
                if (byte == TJC_FRAME_HEAD) {  // 0xDD
                    parseState = ParseState::READ_CMD1;
                }
                // 不是 0xDD 则忽略（可能是噪声或对齐数据）
                break;

            case ParseState::READ_CMD1:
                cmdByte1 = byte;  // 保存命令码第 1 次
                parseState = ParseState::READ_CMD2;
                break;

            case ParseState::READ_CMD2:
                if (byte == cmdByte1) {
                    // 两次命令码一致，校验通过
                    parseState = ParseState::WAIT_TAIL;
                } else {
                    // 命令码不一致，丢弃此帧
                    discardCount++;
                    DebugSerial.printf("[TJC RX] ERROR: cmd mismatch 0x%02X vs 0x%02X\n",
                                       cmdByte1, byte);
                    parseState = ParseState::WAIT_HEAD;
                }
                break;

            case ParseState::WAIT_TAIL:
                if (byte == TJC_FRAME_TAIL) {  // 0xDE
                    // 帧尾匹配，完整帧解析成功
                    cmd = cmdByte1;
                    parseState = ParseState::WAIT_HEAD;
                    return true;
                } else {
                    // 帧尾不匹配，丢弃
                    discardCount++;
                    DebugSerial.printf("[TJC RX] ERROR: tail mismatch, expected 0x%02X got 0x%02X\n",
                                       TJC_FRAME_TAIL, byte);
                    parseState = ParseState::WAIT_HEAD;
                }
                break;
        }
    }

    return false;  // 没有完整帧
}

// ============================================================================
// 向 TJC 发送指令
// ============================================================================

void TjcPanel::sendRaw(const char *cmd) {
    if (cmd == nullptr) return;

    TJC.print(cmd);
    TJC.write(TJC_CMD_END_BYTE);
    TJC.write(TJC_CMD_END_BYTE);
    TJC.write(TJC_CMD_END_BYTE);

    DebugSerial.printf("[TJC TX] %s\n", cmd);
}

void TjcPanel::setText(const char *component, const char *text) {
    if (component == nullptr || text == nullptr) return;

    // 使用 TJC 的文本赋值语法：component.txt="text"
    TJC.print(component);
    TJC.print(F(".txt=\""));
    TJC.print(text);
    TJC.print(F("\""));
    TJC.write(TJC_CMD_END_BYTE);
    TJC.write(TJC_CMD_END_BYTE);
    TJC.write(TJC_CMD_END_BYTE);

    DebugSerial.printf("[TJC TX] %s.txt=\"%s\"\n", component, text);
}

// ============================================================================
// 预留 UI 刷新（当前为空实现）
// ============================================================================

void TjcPanel::refreshReservedStatus() {
    // 预留：后续可在此处批量刷新 TJC 控件
    // 例如：
    //   setText("t_link",  state.vehicleOnline ? "在线" : "离线");
    //   setText("t_obj",   buffer);
    //   setText("t_mode",  modeStr);
    //   ...
}
