# Trigger Summary

当前共有 **6** 种触发方式。触发状态会在 OLED 前两行输出，多个触发用 `|` 连接。

| 触发方式 | OLED 显示 | 条件说明 |
| --- | --- | --- |
| OPEN | `OPEN` | OPEN 常开输入闭合（低电平有效，上拉） |
| CLOSE | `CLOSE` | CLOSE 常闭输入断开（高电平有效，上拉） |
| GAS_D1 | `GAS1` | 传感器 D1 高电平有效（下拉防浮空） |
| GAS_D2 | `GAS2` | 传感器 D2 高电平有效（下拉防浮空） |
| ADC | `ADC` | ADC 采样失败 |
| TEMP | `TMP` | 温度超过 70°C |

补充：
- 任一触发生效时，OLED 第 1 行显示 `[System Action]`，第 2 行显示触发列表（分页显示，每页 16 字符）。
- 无任何触发时，OLED 显示 `System Idle`。
- 预热期倒计时显示：`System warming` / `up T-xxs`。
- `[System Action]` 为锁存状态：一旦触发过会保持，直到重启进入预热期；锁存本身不在 OLED 触发列表中显示。
