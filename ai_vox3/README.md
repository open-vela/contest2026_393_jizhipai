# AI-VOX3 桌面小跟班

> 选题方向：**AI 硬件产品创新**
> 硬件平台：AI-VOX3（ESP32-S3-R8，8 MB PSRAM）+ ES8311 Codec + ST7789 240×240 屏
> 系统：openvela（NuttX 内核）

按住 BOOT 键说话，松手后板子把语音送到 ASR → 大模型 → TTS，再从喇叭回答你；
屏幕上常驻时间、天气与网络状态，并在「聆听 / 思考 / 播报」之间切换状态徽章。

---

## 一、功能

| 编号 | 功能 | 状态 |
|------|------|------|
| F1 | BOOT 键 PTT 录音（按下开始、8 秒自动结束） | ✅ |
| F2 | 16 kHz / 16 bit 单声道 I2S 采集 | ✅ |
| F3 | 批量 ASR（HTTP `POST /asr`） | ✅ |
| F4 | 大模型对话（OpenAI 兼容 `/v1/chat/completions`） | ✅ |
| F5 | TTS 合成并播放（`GET /tts`，audio/L16，16 kHz） | ✅ |
| F6 | 开机问候语自播 | ✅ |
| F7 | ST7789 信息屏：时间 / 天气 / 网络 / 语音状态 | ✅ |
| F8 | 舵机、RGB LED 等外设动作 | ✅ |

服务端点默认走自建服务：ASR `:9009/asr`、TTS `:2301/tts`、LLM `:2302/v1/chat/completions`，
编译期由 `esp32s3_bringup.c` 顶部的宏决定（`/data` 是 tmpfs，运行时改文件无效）。

---

## 二、目录

```
ai_vox3/
├── app/                        作品应用层
│   ├── info_screen/            信息屏：时间 + 天气 + 网络 + 语音状态（直接画 /dev/fb0）
│   ├── day2_ui/                LVGL 界面（含 MiSans 字体）
│   ├── status_indicator/       状态指示
│   ├── my_ai_tool/             自定义 AI 工具示例
│   └── hello_app/
├── packages_ai_agent/          AI Agent 框架（对话编排）
│   └── src/
│       ├── agent_main.c        启动流程 / 端点 seed
│       ├── voice/              采集、播放、ASR、TTS
│       ├── tools/              refresh_weather / get_time / web_search 等工具
│       ├── ui/                 LVGL 通道
│       └── llm/                大模型客户端
└── nuttx/                      对 openvela 内核与板级的修改（核心技术资产）
    ├── arch/xtensa/src/esp32s3/
    │   ├── esp32s3_i2s.c       I2S 驱动（本轮主要战场，见下节）
    │   └── esp32s3_dma.c       GDMA 描述符与通道
    ├── boards/xtensa/esp32s3/esp32s3-eye/
    │   ├── src/esp32s3_bringup.c  板级 bringup、端点 seed、网络保活
    │   └── configs/ai_agent/defconfig
    ├── drivers/audio/es8311.c  ES8311 Codec 驱动
    └── .config                 可用配置快照
```

---

## 三、技术要点（本轮攻克的硬骨头）

### 3.1 I2S 全链路：从「喇叭一声不响」到「能收能放」

`esp32s3_i2s.c` 是本作品改动最深的文件。关键结论全部来自实测，不是推测：

| 问题 | 根因 | 修法 |
|------|------|------|
| 播放第 1 次成功、第 2 次起失败 | ① 诊断代码对已消费描述符反复 re-arm → `OUT_DSCR_ERR`；② `sq_addlast(&tx.act)` 在 `dma_load()` **之后**，GDMA 一 arm 就完成，完成事件找不到 active 条目被丢 | 关掉 re-arm；入队移到 arm 之前 |
| 帧时钟不起 | 官方 `i2s_ll_tx_start()` 是 `tx_update → 等自清 → tx_start`，本驱动写反了 | 调回官方顺序 |
| 录音永远 0 字节（本轮最难） | `i2s_configure()` 只要 TX 占着端口就**无条件**置 `I2S_RX_SLAVE_MOD`，RX 只能从 `I2S0I_BCK_IN/WS_IN` 取时钟；实测把引脚接到这两个外设输入后仍然零数据，连片内 `SIG_LOOPBACK` 回灌都不出 | **让 RX 当主设备**（清 `RX_SLAVE_MOD`，用自己分频器），数据立刻出来：`[RXEOF] first SUC_EOF!` |
| 补丁改了没效果 | 改在 `i2s_rx_channel_start()` 无效——`i2s_configure()` 在它**之后**又跑一遍把 slave 位设回去 | 必须改在 `i2s_configure()` 的判断里 |
| 数据到了上层却读不到 | RX 版的「完成丢失」：`sq_addlast(&rx.act)` 同样在 `dma_load()` 之后 | 用**临界区**把 arm + 入队包成原子操作（单纯调换顺序会导致中断重入崩溃） |
| 收到数据后系统崩溃 | `EXCCAUSE=001c`，`IRQ Stack base 0x3fccb800 size 2048 sp 0x3fccbeb0` → 中断栈只剩 336 字节 | `CONFIG_ARCH_INTERRUPTSTACK` 2048→4096，并关掉中断上下文的诊断打印；DRAM 不够由 `I2S_DMABUF_SLOTS` 4→3 腾出 |
| 开机自检一开就崩（V396） | 扩栈后崩溃点落在 `up_saveusercontext`（剩 336 字节，连保存异常现场都写不下），说明 RX 完成回调路径的中断栈峰值仍然过高 | V396 关闭 `CONFIG_AI_AGENT_VOICE_BOOT_TEST`（开机 RX 自检）。开机不再触发 RX DMA，屏幕正常点亮、系统长时间稳定运行；RX 中断栈的根治（把完成处理下沉到工作队列）列为后续工作 |

> 经验：ESP32-S3 的 I2S 没有 CPU 可写的 FIFO 数据口，音频只能由 GDMA 喂，
> 所以 outlink / inlink 必须修好，没有轮询兜底这条路。

### 3.2 语音链路与网络

- TTS 收不下 81 792 字节的大响应 → 根本原因是 IOB 池容量不足（`64×196` 仅 12.5 KB）。
  扩到 `160×512` 并放大 TCP 相关缓冲后，TTS 从「必失败」变成稳定 4.6 s 完成。
- 端点用内网自建服务，绕开校园网对公网流量的策略路由。
- 禁用 `ping`（在本板上会让芯片崩溃重启），连通性改用固件自带的 `[V58] probe`。

### 3.3 按键

- BOOT = GPIO0（低有效）。GPIO0 的 IO_MUX 会被别的驱动改走，导致 `GPIO_IN_REG` 冻结；
  轮询里每 100 ms 重新 `esp32s3_configgpio(0, INPUT|PULLUP)` 自愈即可。
- 「松手」边沿同样会冻结，改用 `VOICE_PTT_MAX_SEC=8` 自动结束录音，不依赖释放边沿。

### 3.4 屏幕

- 画屏的是 `info_screen`（不是 LVGL，`CONFIG_AI_AGENT_LVGL_UI` 未开）：
  `open("/dev/fb0")` + `ioctl(FBIO_UPDATE)`，开机自启，读 `/tmp/weather.txt`、`/tmp/net.txt`、`/tmp/emotion.txt`。
- 天气来自 `refresh_weather` 工具拉 Open-Meteo，时间来自 NTP + 联网后的 HTTP Date 校准。
- **V396 实测（屏幕已点亮）**：`[LCDDIAG]` 全程走完 `SWRESET → SLPOUT → DISPON → INVON → 满屏填充`，
  背光 GPIO16 保持高电平，`lcdfb` 每次 `putarea` 推送 240×240×2 = 115 200 字节且 `ret=0`，
  `info_screen` 开机自启并持续刷新（`/tmp/net.txt` 周期更新），SNTP 校时成功、已拿到 IP，
  **整个启动过程零 panic**。

---

## 四、编译与运行

```bash
# 在 openvela 工程根目录
./build.sh esp32s3-eye:ai_agent -j8      # 产物 nuttx/nuttx.bin

# 烧录（按住 BOOT 进下载模式，烧完松开 BOOT 重新上电）
esptool --chip esp32s3 --port <COM> -b 460800 \
        --after hard_reset write_flash 0x0 nuttx.bin
```

把本目录按 manifest 映射回 openvela 工程即可直接编译：
`app/` → `apps/` 或 `contest2026_307_justdoit/app/`；
`packages_ai_agent/` → `packages/ai_agent/`；
`nuttx/` 下的文件覆盖同名路径。

> 烧录后芯片常停在 ROM 下载模式导致串口 0 字节，**拔插 USB 冷上电**即可恢复。

---

## 五、许可

Apache-2.0（与 openvela 上游一致）。
