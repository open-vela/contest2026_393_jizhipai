# AI-VOX3 桌面小跟班

队伍：**竞智派（jizhipai）** · 队伍编号 **393** · 仓库 `contest2026_393_jizhipai`

## 一、作品简介

基于 openvela（NuttX）在 **AI-VOX3（ESP32-S3-R8 + ES8311 + 240×240 ST7789 LCD）** 上做的
一款可完全自主掌控的桌面语音对话终端：**按键即可完成「录音 → ASR → LLM → TTS → 播放」的完整对话**，
同时在同一块屏上常驻显示时间、天气与网络状态，底部保留语音字幕。

与闭源商用音箱的区别：代码全部开放、可二次开发、不绑定厂商云生态，
并且针对**校园网（存在策略路由与专网出口）**做了公网 / 内网双后端降级。

主要工作量在系统侧适配与修复：

- 修复 ESP32-S3 **I2S 单端口全双工**时钟配置（TX 占用端口时 RX 被强制设为从模式，
  而从模式所需的外部时钟并未送达 RX，导致录音长期返回 0 字节）
- 补齐 RX **TDM 通道数**配置
- 修复 **GDMA 完成队列竞态**（「装载 DMA → 启动 → 挂入完成队列」三步放进临界区，
  否则传输在挂队之前完成时该帧事件永久丢失）
- **IOB 缓冲池**由 64×196 扩到 160×512，网络就绪时间 **124 s → 5 s**
- 定位 RX 首帧到达后的崩溃为 **WiFi 闭源 FIQ 与 GDMA RX 中断的并发冲突**
  （`EXCCAUSE=001c` / `VADDR=0`；已用三档中断栈对照排除栈容量因素，实测峰值仅 336 B）

## 二、选题方向

**AI 硬件产品创新 + 新硬件平台适配**（两项组合）。

- AI 硬件产品创新：语音对话 + 桌面信息屏的完整产品形态，含自定义工具与 Skill。
- 新硬件平台适配：AI-VOX3 板级的 LCD（SPI3）、ES8311（I2C0）、I2S / GDMA 全链路驱动适配与修复。

## 三、目录结构

```text
ai_vox3/
├── app/                     自研应用
│   ├── info_screen/         信息屏（时间 / 天气 / 网络 / 语音字幕，3 页可切换）
│   ├── status_indicator/    状态指示与状态条
│   ├── day2_ui/             对话界面（LVGL）
│   ├── my_ai_tool/          示例自定义工具
│   └── hello_app/           示例应用
├── packages_ai_agent/       ai_agent 框架（语音 / 网络 / 工具 / Skill 运行时）
└── nuttx/                   内核与板级改动
    ├── arch/xtensa/src/esp32s3/              I2S、GDMA 驱动修复
    ├── boards/xtensa/esp32s3/esp32s3-eye/    板级配置与 bringup
    └── drivers/audio/es8311.c                编解码器
docs/                        技术文档（含 V399–V401 RX 崩溃排查复盘）
logs/                        AI Coding 日志
```

## 四、运行方式

### 1. 拉取工程

```bash
repo init -u https://github.com/open-vela/contest2026_393_jizhipai \
  -b dev-ai-contest-2026 -m contest2026_393_jizhipai.xml
repo sync -c -j8
```

### 2. 编译

在 openvela 工作区根目录（本仓的上一级）：

```bash
./build.sh vendor/openvela/boards/contest2026_393_esp32s3_eye/configs/ai_agent -j8
```

产物：`nuttx/nuttx.bin`

### 3. 烧录

按住 **BOOT（GPIO0）** 上电进入下载模式：

```bash
python -m esptool --chip esp32s3 --port <COM> write_flash 0x0 nuttx.bin
```

烧完**松开 BOOT 并拔插 USB**（否则芯片会停在 ROM 下载模式，串口 0 字节）。

### 4. 运行

- 上电约 35 s 后自播开机问候语「桌面小跟班语音就绪」。
- 按 **BOOT** 开始说话（松手或 8 s 自动结束录音）→ ASR → LLM → TTS 回播。
- **VOL−（GPIO45）上一页 / VOL+（GPIO46）下一页**，共 3 页：
  首页概览 / 网络详情 / 系统详情，右上角显示 `n/3`。
- 也可以直接说「下一页」，由自定义工具 `ui_control` 下发 `next` / `prev` / `home`。
- 语音字幕在所有页面底部常驻。

### 5. 硬件接线（GPIO）

| 功能 | 引脚 |
| --- | --- |
| I2S MCLK / BCLK / WS | 11 / 10 / 8 |
| I2S DOUT / DIN | 7 / 9 |
| ES8311 I2C SDA / SCL | 13 / 12 |
| LCD MOSI / CLK / DC / CS / BCKL | 21 / 17 / 14 / 15 / 16 |
| BOOT（PTT） | GPIO0 |
| VOL+ / VOL− | 46 / 45 |

### 6. 服务端端点

语音与对话走云端，端点在编译期 seed 中（`esp32s3_bringup.c`、`http_tts.c`、`http_asr.c`）：

- TTS：`http://42.48.185.89:2301/tts`
- ASR：`http://42.48.185.89:9009/asr`
- LLM：`http://42.48.185.89:2302/v1/chat/completions`
- 内网备选：`http://172.16.88.1` 同端口

> `/data` 在本平台是 tmpfs，运行时改文件无效，端点只能改编译期 seed。

## 五、AI Coding 使用说明

本作品全程借助 AI（WorkBuddy / CodeBuddy Agent）辅助开发，AI 参与率约 70%：

- **需求拆解与方案设计**：由 AI 从「能语音对话的桌面终端」拆成采集 / 传输 / 合成 / 播放 / 显示五段，
  并逐段设计可判定的验证方法。
- **编码**：驱动修复（I2S 时钟、GDMA 临界区、IOB 与中断栈参数）与应用逻辑
  （信息屏、UI 状态机、自定义工具）均由 AI 产出补丁脚本后逐版验证。
- **调试**：崩溃定位从「人工读几千行串口日志」变成「关键字统计 + addr2line」，
  一次定位从小时级压到分钟级。
- **文档**：技术报告与本 README 由 AI 依据仓库与日志事实生成，保证与代码一致。

踩过的坑（已固化到排障纪律）：

1. AI 一度把故障归因于网络环境，后用「同板同网、UDP 通而 TCP 不通」的对照实验推翻，
   改从软件栈容量入手。
2. AI 连续三版按「中断栈不足」扩容无效，后改为**先测量再下结论**（实测峰值 336 B）
   才转向并发方向。
3. 外部包 Kconfig 符号（`CONFIG_AI_AGENT_VOICE_BOOT_TEST`）会被 nuttx `olddefconfig`
   静默丢弃，`#ifdef` 包住的代码从未编译 → 此后一律改为显式宏门控并在 ELF 中校验符号。

完整对话日志见 `logs/` 目录。

## 六、已知不足与后续

1. RX 中断与 WiFi 闭源 FIQ 的并发冲突尚未根治（当前用编译期开关 `CAPTURE_PROBE` 规避，
   置 1 可复现）；下一步在 `xtensa_user_panic` 之前打点抓第一手异常，
   并尝试在 RX 中断期间临界区屏蔽 WiFi FIQ 验证假设。
2. 唤醒词尚未接入，当前为按键 PTT。
3. 未做系统级低功耗优化。
