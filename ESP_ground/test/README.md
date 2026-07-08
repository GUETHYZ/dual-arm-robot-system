# 测试说明

## 当前状态

本目录预留给后续单元测试和集成测试。

## 建议测试项

1. **CRC16 单元测试**：验证 `crc16Modbus()` 对已知输入的计算结果。
2. **协议打包/解析往返测试**：打包一个帧再用解析器解析，验证一致性。
3. **TJC 按钮帧解析测试**：喂入合法/非法帧，验证状态机行为。
4. **命令映射测试**：验证所有 TJC 命令码正确映射到协议功能码。

## 使用方式

可在 PlatformIO 中添加 `test/` 目录的测试配置：

```ini
[env:native]
platform = native
test_framework = unity
```

或使用 Arduino 的 `Serial.printf` 调试输出手动验证。
