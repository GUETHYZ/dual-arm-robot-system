/**
 * @file CommandMapper.cpp
 * @brief TJC 按钮命令码到统一协议功能码的映射实现
 *
 * 映射关系（必须与 docs/通信协议规范.md 一致）：
 *
 *   TJC cmd | 来源按钮            | 统一协议功能码
 *   --------|---------------------|------------------
 *   0x01    | b_forward 按下       | CMD_CHASSIS_FORWARD
 *   0x02    | b_back 按下          | CMD_CHASSIS_BACKWARD
 *   0x03    | b_right 按下         | CMD_CHASSIS_RIGHT
 *   0x04    | b_left 按下          | CMD_CHASSIS_LEFT
 *   0x05    | b_clamp 按下         | CMD_ARM_BOTH_PICK
 *   0x06    | b_put_down 按下      | CMD_ARM_BOTH_PLACE
 *   0x10    | 移动按钮弹起 / b_stop | CMD_CHASSIS_STOP
 *   0x11    | b_emergency          | CMD_EMERGENCY_STOP
 *   0x12    | b_arm_stop           | CMD_ARM_STOP
 */

#include "app/CommandMapper.h"
#include "protocol/Protocol.h"
#include "config/HardwareConfig.h"
#include <Arduino.h>

// ============================================================================
// 映射表项
// ============================================================================

struct TjcMappingEntry {
    uint8_t tjcCmd;       ///< TJC 按钮事件命令码
    uint8_t funcCode;     ///< 统一协议功能码
    const char *name;     ///< 按钮/命令名称
};

/// TJC 命令码 → 统一协议功能码的完整映射表
static const TjcMappingEntry mappingTable[] = {
    // 移动类按钮——按下事件
    { 0x01, CMD_CHASSIS_FORWARD,  "b_forward"   },
    { 0x02, CMD_CHASSIS_BACKWARD, "b_back"      },
    { 0x03, CMD_CHASSIS_RIGHT,    "b_right"     },
    { 0x04, CMD_CHASSIS_LEFT,     "b_left"      },

    // 机械臂动作按钮——按下事件
    { 0x05, CMD_ARM_BOTH_PICK,    "b_clamp"     },
    { 0x06, CMD_ARM_BOTH_PLACE,   "b_put_down"  },

    // 停止类按钮——按下事件 / 移动按钮弹起事件
    { 0x10, CMD_CHASSIS_STOP,     "b_stop"      },

    // 预留按钮
    { 0x11, CMD_EMERGENCY_STOP,   "b_emergency" },
    { 0x12, CMD_ARM_STOP,         "b_arm_stop"  },
};

static constexpr size_t mappingTableSize = sizeof(mappingTable) / sizeof(mappingTable[0]);

// ============================================================================
// 映射查找
// ============================================================================

static const TjcMappingEntry* findMapping(uint8_t tjcCmd) {
    for (size_t i = 0; i < mappingTableSize; i++) {
        if (mappingTable[i].tjcCmd == tjcCmd) {
            return &mappingTable[i];
        }
    }
    return nullptr;
}

// ============================================================================
// 公共接口
// ============================================================================

bool CommandMapper::mapTjcCommand(uint8_t tjcCmd, GroundCommand &outCmd) {
    const TjcMappingEntry *entry = findMapping(tjcCmd);

    if (entry == nullptr) {
        DebugSerial.printf("[MAPPER] unknown TJC cmd: 0x%02X\n", tjcCmd);
        return false;
    }

    // 填充输出命令
    outCmd.funcCode = entry->funcCode;
    outCmd.dataLen  = 0;  // 控制命令通常无数据区

    DebugSerial.printf("[MAPPER] %s (0x%02X) → %s (0x%02X)\n",
                       entry->name, tjcCmd,
                       funcCodeToString(entry->funcCode), entry->funcCode);

    return true;
}

const char* CommandMapper::tjcCmdToName(uint8_t tjcCmd) {
    const TjcMappingEntry *entry = findMapping(tjcCmd);
    return (entry != nullptr) ? entry->name : "UNKNOWN";
}

const char* CommandMapper::funcCodeToName(uint8_t funcCode) {
    return funcCodeToString(funcCode);
}
