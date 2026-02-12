# TODO（业务逻辑拆解）

## 目标说明
- 核心逻辑：当任一核心输入异常（模拟量越界 / 内部温度越界 / 数字输入触发）时，拉高 `OutEnable0` 和 `OutEnable1`。
- 启动保护：上电/复位后的前 `15s` 内，**禁止触发**上述拉高逻辑。
- 开发优先级：先完成核心逻辑，再补充 I2C、LED、按键、串口指示灯等辅助功能。

## 核心功能 I/O 定义（按代码 Tag）
- 输出：
  - `OutEnable0`
  - `OutEnable1`
- 输入：
  - `AnalogInput0`
  - `AnalogInput1`
  - `AnalogInput2`
  - `ADC 内部温度`（内部温度通道）
  - `DigitalInput0`
  - `DigitalInput1`
  - `DigitalInput2`
- 数字输入策略：
  - `DigitalInput0~2` 使用 `GPIO_MODE_INPUT`，在主循环周期读取引脚电平，不使用 EXTI 中断。
  - 原因：实时性要求不高，但需要稳定性（可在软件层增加连续采样判稳/去抖）。

## 阶段1：核心逻辑（优先）
- [x] 定义阈值参数：`ADC_MIN/ADC_MAX`、`TEMP_MIN/TEMP_MAX`（当前已用宏占位，后续按实测替换）。
- [x] 建立启动计时：使用 `HAL_GetTick()` 生成 `boot_guard_active = (tick < 15000)`。
- [x] 完成 ADC 采样通路：`AnalogInput0~2` 与内部温度，周期获取最新值。
- [x] 完成数字输入采集：主循环读取 `DigitalInput0~2`，并加入基础判稳（N 次一致才生效）。
- [x] 新增统一判定函数（`CoreProtection_Update()`）：
  - [x] 若 `boot_guard_active == true`，则不允许触发 `OutEnable0/OutEnable1`。
  - [x] 若 `boot_guard_active == false` 且任一条件满足（模拟量越界/温度越界/数字输入触发），则置 `OutEnable0 = HIGH` 且 `OutEnable1 = HIGH`。
- [x] 明确 `OutEnable0/OutEnable1` 行为：触发后锁存到复位，不做自动切换。
- [x] 在主循环中固定周期执行判定，避免抖动（已实现周期调度与数字输入判稳）。

## 阶段2：验证与稳定性
- [ ] 用串口打印关键状态：`tick`、`AnalogInput0~2`、温度值、`DigitalInput0~2`、`OutEnable0/OutEnable1` 状态。
- [ ] 编写最小化测试清单：
  - [ ] 0~15s 内制造异常，确认 `OutEnable0/OutEnable1` 不拉高。
  - [ ] >15s 后分别触发模拟量越界、温度越界、数字输入触发，确认 `OutEnable0/OutEnable1` 拉高。
  - [ ] 边界值测试（刚好 15000ms、阈值上下边界）。
- [ ] 若出现误触发，加入滤波/连续超限计数策略。

## 阶段3：辅助功能（核心完成后）
- [ ] I2C 外设接入与通信自检。
- [x] LED 状态指示：
  - [x] LED0：系统状态，500ms 翻转一次（心跳）。
  - [x] LED1：触发指示，触发后 100ms 翻转快闪（锁存到复位）。
- [ ] 按键功能（状态清除、模式切换或调试触发）。
- [ ] 串口指示灯或串口日志增强（便于联调）。

## 待确认问题
- [x] `OutEnable0/OutEnable1` 触发后锁存到复位。
- [ ] ADC/温度“超过一定范围”是上限、下限还是双向越界？
- [ ] `DigitalInput0~2` 的触发电平定义（高有效/低有效）与判稳窗口长度。
