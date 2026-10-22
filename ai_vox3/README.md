# AI-VOX3 桌面小跟班（桌面伴侣）

> 选题方向：**AI 硬件产品创新**
> 硬件平台：AI-VOX3（ESP32-S3-R8，8 MB PSRAM）+ ES8311 Codec + ST7789 240×240 LCD + 双舵机云台 + K230 视觉协处理器
> 系统：openvela（NuttX 内核）+ ai_agent 框架 + 小米 MiMo 大模型

---

## 一、作品简介

按住 BOOT 键说话，松手后板子把语音送上 ASR → 大模型 → TTS，从喇叭回答你；屏幕上常驻时间、天气与网络状态，并在「聆听 / 播报」之间切换状态徽章。在此之上，本轮（决赛阶段）新增了**双舵机云台 + K230 视觉协处理器**：K230 跑人脸检测，通过 UDP 把人脸中心的千分比坐标发给 ESP32，ESP32 解算后驱动 LEDC PWM 让云台跟随人脸转动——**设备不再只是"会说话的盒子"，而是会看着你的桌面伴侣**。

与闭源商用音箱的区别：代码全部开放、可二次开发、不绑定厂商云生态，并且针对校园网（存在策略路由与专网出口）做了公网 / 内网双后端降级。

主要工作量在系统侧适配与修复：ESP32-S3 的 I2S 单端口全双工时钟、GDMA 完成队列竞态、IOB 网络缓冲池、抖动缓冲播放调度、LEDC 舵机引脚映射、以及 K230 跨设备联调。

---

## 二、硬件清单与接线

| 部件 | 规格 | 接口 |
| --- | --- | --- |
| 主控 | AI-VOX3（ESP32-S3-R8，8 MB PSRAM 已并入堆） | — |
| 音频 Codec | ES8311，16 kHz / 16 bit 单声道 | I2C0 + I2S |
| 显示屏 | ST7789 240×240 | SPI3 @ 20 MHz |
| 云台 | 2 × 模拟舵机（50 Hz，脉宽 1000–2000 µs） | LEDC PWM |
| 视觉协处理器 | K230（CanMV），跑人脸检测 | WiFi，UDP 单播 |

### GPIO 分配

| 功能 | 引脚 |
| --- | --- |
| I2S MCLK / BCLK / WS | 11 / 10 / 8 |
| I2S DOUT / DIN | 7 / 9 |
| ES8311 I2C SDA / SCL | 13 / 12 |
| LCD MOSI / CLK / DC / CS / BL | 21 / 17 / 14 / 15 / 16 |
| 舵机 PAN（水平） | **GPIO2**（`/dev/pwm0`，LEDC chan0） |
| 舵机 TILT（俯仰） | **GPIO1**（`/dev/pwm1`，LEDC chan1） |
| BOOT（PTT） | GPIO0（低有效） |
| VOL+ / VOL−（翻页） | 46 / 45 |

> 注意：GPIO4 在本板是**台灯继电器**，不是舵机引脚。历史版本曾把俯仰轴错配到 GPIO4，详见第七节 7.5。

### 网络拓扑

```
K230 (192.168.110.12)  --UDP:3333-->  ESP32-S3 (192.168.110.154)  --WebSocket--> 云端 ASR/LLM/TTS
        摄像头                            STA，连路由器 SSID
```

K230 脚本里硬编码了目的 IP `192.168.110.154`；ESP32 侧 DHCP 拿到的是 `.155`，因此在应用层 `ensure_dns()` 内把地址覆盖回 `.154`（网关 `192.168.110.1`，掩码 /24），先 DHCP 保证联网、再覆盖成 K230 期望的地址，不动 Kconfig，风险可控且可逆。

---

## 三、软件架构

```
用户交互层     BOOT 按键 PTT / VOL+ VOL− 翻页 / LVGL 触摸 / NSH CLI
                    |
功能层         语音对话 · K230 人脸跟踪云台 · 信息屏 · 工具调用（MCP）
                    |
运行时         ai_agent：agent_main / voice / tools / ui / llm / infra(network_manager)
               ai_vox：实时 WebSocket 语音链路（Opus 上下行）
                    |
驱动层         ES8311 (/dev/audio/pcm0p/pcm0c) · I2S+GDMA · LEDC PWM (/dev/pwm0,1)
               SPI3 + ST7789 (/dev/fb0) · WiFi STA
                    |
内核           openvela / NuttX
```

**两条语音链路并存**：

- **ai_vox 链路**：WebSocket 全双工，上行 ES8311(16 kHz) → I2S → `ReadPcm(960)` → Opus(24 kbps)，下行 Opus 解码 → 重采样 → 抖动缓冲 → I2S TX → ES8311 DAC。实测 9 轮对话全部走这条链路，负责"能对话"这件事本身。
- **ai_agent 本地链路**：MiMo 大模型 + MCP 工具（天气、米家设备控制等）+ `voice_channel_speak()` 播报，负责"能做事"。

**K230 云台链路**：`companion_face.c` 起独立线程，`bind(0.0.0.0:3333)` 收 UDP → 解析 → EMA 平滑 → 绝对角度映射 → `servo_move_pan/tilt()` 记账 → 200 ms 保活线程真正写 PWM。

---

## 四、功能演示说明

### 开机（这一步很关键）

```
拔 USB → 插 USB → 【按开机键】→ 等 10 s
```

面板的供电靠开机键的**硬件 latch**，不是固件控制的 GPIO。不按开机键 = 面板没电 = 屏幕全黑，但串口日志一切正常。

### 演示 1：语音对话

1. 按住 **BOOT** 说话，松手或 8 s 自动结束录音。
2. 屏幕切换为「开始聆听」→ 服务端生成回复 → 「播报」并出声。
3. 答完自动进入续听（推挽式连续对话窗），可继续追问。
4. 说「今天天气怎么样」触发 MCP 天气工具；米家设备控制已接入 MCP，需用户自备凭据后可用。

### 演示 2：K230 人脸跟踪云台

1. 站到 K230 摄像头前，云台会朝人脸方向转动；人脸在画面左右移动时水平轴跟随，上下移动时俯仰轴跟随。
2. 人离开画面 → K230 持续发 `N` → 约 0.9 s 后云台平滑回中（不是硬拽，有限速）。
3. 控制律为**开环绝对映射**：摄像头固定在桌面，云台转动不改变摄像头画面，因此不存在反馈通路，不会过冲也不会自激。

### 演示 3：屏幕与翻页

- 按 **VOL−（GPIO45）/ VOL+（GPIO46）** 翻页，共 3 页：首页概览 / 网络详情 / 系统详情，右上角显示 `n/3`。
- 也可以直接说「下一页」，由自定义工具 `ui_control` 下发 `next` / `prev` / `home`。

---

## 五、编译与烧录

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

把本目录按 manifest 映射回 openvela 工程即可直接编译：`app/` → `apps/`；`packages_ai_agent/` → `packages/ai_agent/`；`nuttx/` 下的文件覆盖同名路径。

> 改 `.c`/`.cpp` 后若产物没变，先 `touch` 再构建；确认新代码进 ELF 用
> `nm nuttx/nuttx | grep <符号>`（static 为小写 `t`，全局为大写 `T`）。

### 3. 烧录

```bash
python -m esptool --chip esp32s3 -p <COM> -b 460800 \
       --before default_reset --after hard_reset write_flash 0x0 nuttx.bin
```

- **只写 `0x0`，绝不碰 `0x9000`**（该分区含 NVS）。
- **烧录前先停掉串口抓包任务**，否则 COM 口被占用。
- 烧完**松开 BOOT 并拔插 USB**（否则芯片会停在 ROM 下载模式，串口 0 字节）。

### 4. 服务端端点

语音与对话走云端，端点在编译期 seed 中（`esp32s3_bringup.c`、`http_tts.c`、`http_asr.c`）：

- TTS：`http://42.48.185.89:2301/tts`
- ASR：`http://42.48.185.89:9009/asr`
- LLM：`http://42.48.185.89:2302/v1/chat/completions`
- 内网备选：`http://172.16.88.1` 同端口

> `/data` 在本平台是 tmpfs，运行时改文件无效，端点只能改编译期 seed。

---

## 六、目录结构说明

```text
ai_vox3/
├── app/                         自研应用
│   ├── info_screen/              信息屏：时间 / 天气 / 网络 / 语音字幕（直画 /dev/fb0）
│   ├── status_indicator/         状态栏与设备状态页
│   ├── day2_ui/                  LVGL 界面（MiSans 字体、弹窗、提醒页、时钟）
│   └── my_ai_tool/               自定义工具示例
├── packages_ai_agent/            ai_agent 框架
│   ├── src/
│   │   ├── agent_main.c          启动流程 / 端点 seed
│   │   ├── companion_face.c/.h   ★ K230 人脸跟踪：UDP 收包、解析、控制律（本轮新增）
│   │   ├── servo.c/.h            ★ 双轴舵机：LEDC PWM、分拍步进、保活线程
│   │   ├── companion_mood.c      触摸触发的点头 / 摇头动作
│   │   ├── voice/                采集、播放、ASR（http/mimo/volc）、TTS
│   │   ├── tools/                MCP 客户端 / 服务端、天气、米家、定时等工具
│   │   ├── llm/                  大模型代理与路由（MiMo 等）
│   │   ├── ui/lvgl_ui_channel.c  LVGL 通道
│   │   ├── infra/network_manager.c  联网、DNS、★ 静态 IP 覆盖
│   │   └── core/                 agent 主循环、上下文构建、记忆、消息总线
│   └── agent_skills/             Skill 模板（天气、提醒、设备诊断、健康等）
└── nuttx/                        内核与板级改动（核心技术资产）
    ├── arch/xtensa/src/esp32s3/
    │   ├── esp32s3_i2s.c         I2S 驱动（全双工时钟、light/full DMA 路径）
    │   ├── esp32s3_dma.c         GDMA 描述符与通道
    │   ├── esp32s3_ledc.c        ★ LEDC 通道→引脚映射（舵机 PAN/TILT 在此强制赋值）
    │   ├── esp32s3_spi.c         SPI3（LCD）
    │   └── esp32s3_wifi_adapter.c / esp32s3_wlan.c
    ├── drivers/audio/es8311.c    Codec 驱动
    ├── drivers/lcd/lcd_framebuffer.c
    ├── boards/.../esp32s3_bringup.c   板级 bringup、端点 seed、网络保活
    └── .config                   可用配置快照
docs/   技术文档（含 V399–V401 RX 崩溃排查复盘）
logs/   AI Coding 日志
```

---

## 七、关键技术攻关

### 7.1 语音播报卡顿：抖动缓冲的两处硬伤

用户描述的"卡、发沙、播几个字就断"曾先后被归因为削波、重采样、I2S 每 buffer 复位、上行 CPU 争抢等，均被硬数据排除（1 kHz 阶梯正弦听测全程纯净；抓取设备真实送出的 PCM 重建 WAV 后用户判"非常完美"；逐包 Opus 记账显示解码输出正常）。最终用**倍率表**（音频时长 ÷ 墙钟时间）定位：

| 版本 | 倍率 | 说明 |
| --- | --- | --- |
| 修复前 | 0.13（6.04 s 只播 788 ms 音频） | 慢 7.7 倍 |
| 修复后 | 0.86 ~ 1.17，中位 ≈ 1.0 | 9 轮实测，`cushion dry` 0 次 |

**硬伤 A —— 排空门槛写错，余量为 0。** 稳态排空条件是 `s_jb >= reserve + CHUNK`，而一次交接只写 CHUNK（2048 样本 = 128 ms），交接后立刻低于自己的门槛 ⇒ 播放停等补满 128 ms。抗抖动余量为 0，任何到达空洞都卡满一整块。改为 `s_jb > reserve && s_jb >= CHUNK`（启动门槛不变，仍先攒够 cushion）。

**硬伤 B —— 服务端不保证发 `tts/stop`。** 实测一轮对话 `sentence_start` 8 次、`tts/stop` **0 次** ⇒ `NotifyDataEnd()` 从不执行 ⇒ 按设计保留的 reserve 永远留在缓冲里 ⇒ 播几个 chunk 就停。原有的 1 s 空闲判定只在"新包到达时"执行，服务端安静了就没人计算空闲时间。修法是加**空闲看门狗**：独立线程每 100 ms 检查，空闲超过 1000 ms 就向 AudioOutput 的任务队列投递 flush（投递到队列而非直接调用，保证对 `s_jb` 的读写仍在同一线程上，无锁无 data race）。

> 两条可复用教训：① **reserve 是要"保留"的量，不是"必须被超过"的量**；② **判据必须由时钟驱动，不能由下行驱动。**

### 7.2 与小智固件的对比：真差异在 DMA

小智的 `audio_output_engine.cpp` 只有 68 行，没有抖动缓冲也没有 pacing，却不卡。逐项对比后确认**线程结构两边完全一致**（WS 回调只 memcpy+Enqueue → 引擎任务队列 → 播放线程），线程优先级、麦克风增益、I2S 主从、解码阻塞等候选全部证伪。

| | 小智 | openvela |
| --- | --- | --- |
| DMA | GDMA **环形描述符链**（`dma_desc_num=4`），**硬件自循环** | 8 个独立 apb（每个 64 ms），**软件交接** |
| 播放引擎 | 68 行，无缓冲 | 抖动缓冲 + pacing + cushion |
| 音量 | 平方律 `pow(v/100,2)*65536` | 已对齐为平方律 |

**结论：小智敢不做缓冲，是因为硬件兜底——软件晚喂，DMA 继续跑上一圈，不中断。openvela 必须靠软件补，因此缓冲参数必须大于网络空洞。** 这是本平台所有音频调参的第一性原则。

### 7.3 LCD 全黑：不是固件，是电源 latch

屏幕全黑时串口日志一切正常（背光 GPIO、SPI 推送、`FBIO_UPDATE` 全部 `ret=0`）。根因是**面板供电靠开机键的硬件 latch**，不按开机键面板就没电。此前误判为"USB CDC TX 环写满死锁导致黑屏"，投入过资源。判据：**软件侧全绿而物理现象为零时，先查电。**

### 7.4 I2S / GDMA 与网络栈容量（初赛阶段，保留）

| 问题 | 根因 | 修法 |
| --- | --- | --- |
| 录音永远 0 字节 | `i2s_configure()` 只要 TX 占着端口就无条件置 `I2S_RX_SLAVE_MOD`，而外部时钟并未送达 RX | 让 RX 当主设备（清 `RX_SLAVE_MOD`，用自己分频器） |
| 播放第 1 次成功、第 2 次起失败 | 诊断代码反复 re-arm 已消费描述符 → `OUT_DSCR_ERR`；`sq_addlast(&tx.act)` 在 `dma_load()` 之后，GDMA 一 arm 就完成，事件找不到 active 条目被丢 | 关掉 re-arm；入队移到 arm 之前 |
| 收到数据后系统崩溃 | `EXCCAUSE=001c`，中断栈只剩 336 字节（三档扩容对照实测） | `CONFIG_ARCH_INTERRUPTSTACK` 2048→4096，`I2S_DMABUF_SLOTS` 4→3 腾 DRAM |
| TTS 收不下 81 792 字节 | IOB 池只有 `64×196`（12.5 KB） | 扩到 `160×512`，网络就绪时间 **124 s → 5 s** |

> 经验：ESP32-S3 的 I2S 没有 CPU 可写的 FIFO 数据口，音频只能由 GDMA 喂，所以 outlink / inlink 必须修好，没有轮询兜底这条路。

### 7.5 K230 人脸跟踪云台：从"240 s 零数据"到全链路打通

这是本轮最曲折的一段，中间的几次误判都值得记录。

**（1）线程被阻塞 socket 卡死。** `cf_open_socket()` 之后没有设 `O_NONBLOCK`，`cf_poll_socket()` 的 `for(;;) { recvfrom(); if(n<0) break; }` 在没包时永远卡在 `recvfrom`，控制律、舵机下发、健康报告**全都不执行**。加 `fcntl(fd, F_SETFL, fl | O_NONBLOCK)` 后主循环才真正按 20 Hz 跑。
> 教训：**"探针无输出"有两种截然不同的原因——线程卡死 vs 数据没来——必须先区分。**

**（2）舵机轴接反，且俯仰轴接到了台灯继电器上。** 真正的引脚在 `esp32s3_ledc.c` 被历史补丁 `[SERVO-R39]` 硬编码为 `ch[0].gpio=1`（PAN）、`ch[1].gpio=4`（TILT），而 GPIO4 是台灯继电器——**改之前"俯仰"通道一直在往继电器灌 50 Hz PWM**。已按用户实测接线改为 **PAN=GPIO2 / TILT=GPIO1**（`[V1041] SUPERSEDES [SERVO-R39]`）。
> 为什么只能在 C 里强制赋值：改 `.config` 会重跑 `olddefconfig` → `clean_context` → 删掉 `chip/esp-hal-3rdparty`，当年 R39 正是这么干留下的坑。

**（3）K230 的目标 IP 靠"读它的脚本"才拿到。** ESP32 侧 `bind(0.0.0.0:3333)` 来者不拒，对端 IP 在 ESP32 上根本不存在，只能硬编码在 K230 脚本里。先后用三种手段探测，全部落空：

- **ARP 观察器**（改 `nuttx/net/arp/arp_input.c`）：450 s 零命中。**盲点**：若目标 IP 当前在线，K230 的 ARP 缓存是热的，不再发 ARP 请求，单播 UDP 直达，ESP32 完全看不见 ⇒ 零命中只能排除"目标离线"。
- **网段扫描**（UDP 探测包逼对方 ARP）：13 台在线，无一像 K230。
- **WiFi 扫描**：只有 3 个 SSID，BSSID 只差第 4 字节（a4/84/f4），是同一台路由器的虚拟 SSID ⇒ K230 若开热点不可能看不见。

最后发现 **K230 根本不在网上**——它用 USB 插在电脑上（COM33，`VID:PID=1209:ABD1`）。它的 USB 是复合设备，其中一个接口是 MTP，SD 卡可像 U 盘一样读（走 `Shell.Application` 的 COM 接口），`xz_face.py:20` 写着：

```python
VOX3_IP   = "192.168.110.154"
VOX3_PORT = 3333
```

**ESP32 是 `.155`（DHCP），K230 发 `.154`，差 1。** 注释里的 MAC 正是这块板，说明它当初就是 `.154`。在应用层覆盖静态 IP 后，立刻收到真实数据：**1154 条报文（F 包 1028 / N 包 118），源 IP `192.168.110.12`**。

> 附带教训：**COM33 串口静默 ≠ K230 没在跑**——它的日志走 UDP 发到 PC 上的另一个地址，且实际运行的脚本是 HTTP 拉下来的版本（`DEBUG=False`）。

**（4）舵机的"两个真值源"隐患（已修）。** `servo_move()` 只写 `g_limits[].target`（记账），真正的 PWM 写入只发生在 200 ms 保活线程里，任何绕开它的直接写入会在 200 ms 内被拉回旧 target。修法是在 `servo_set_ch()` 末尾同步 `g_limits[ch->idx].target`；但**不能无条件同步**——分拍步进若同步 target 会把长行程截断在第一拍（budget 36°/拍，cur=0 想到 109 会永久停在 36），故保活的两类写入走独立的 `servo_set_ch_keep_target()` 变体。

### 7.6 控制律与增益标定

```c
pan  = 90 + (cx - 500) * 0.16f    /* 限幅 10..170 */
tilt = 90 - (cy - 500) * 0.14f    /* 限幅 20..160 */
```

`cx/cy` 是人脸中心的千分比坐标（0..1000）。增益与限幅精确配套：`500‰ × 0.16 = 80°`，`90 ± 80 = 10..170` 正好等于限幅边界；舵机层自身限幅 5..175，**安全限位未被突破**。实测 **pan 跨度 72°、tilt 跨度 99°**，脏数据告警 0。

调参过程同样靠量化而非感觉：

- **"跟踪太慢"**：先查快照发现 `cur` 与 `tgt` 始终相等 ⇒ 舵机到位了，慢的是目标角本身。真正的闸门不是死区常量，而是硬编码的 `>2°` 判定，以及 ESP32 侧与 K230 侧的**两级平滑叠加**（K230 `SMOOTH_ALPHA=0.5` + ESP32 EMA alpha 0.375）。改法：EMA alpha 0.375→**0.75**、限速 3→**12**（60°/s→240°/s）、丢失回中 2000→**900 ms**；死区只压到 2.0° 就停（舵机模块自身有 2° 死区，再压只会产生被静默丢弃的目标角）。
- **"幅度太小"**：量化发现 clamp 只用了 1/4（pan 63~93）和 1/8（tilt 93~108）⇒ 不是 bug 是增益保守。原增益 0.065 / 0.051 是按物理视场角算的**精确对准值**，但作为"伴侣"手感太弱，放大到 0.16 / 0.14。
- **顺带堵住一个真漏洞**：串口串扰曾产生 `cx=3077`，而跳变拒绝阈值会随人脸宽度放宽到 600，`clampi` 会把它折成合理的边缘值 1000 ⇒ 增益放大后会直接撞限位。已加第三道校验：`cx/cy` 超出 0..1000 **整帧丢弃**（不夹紧，因为整行已坏）。

### 7.7 内存：DRAM 只剩 112 字节

```
dram0_0_seg: org=0x3fc88000 len=0x48000 → 结束 0x3FCD0000
_edata/_sheap = 0x3fccff90   剩余 = 0x70 = 112 B
```

实测一个 24×16 B 的静态数组（384 B）就会让链接器报 `region dram0_0_seg overflowed by 272 bytes`。因此本项目所有调试样本窗口一律放**栈上或用堆**（PSRAM 已并入堆，`malloc` 可拿几十 KB），大常量数组放 flash。**判断余量要看 `_edata`，不能看 elf-size 的 data/bss 增减**——删掉中段字符串后 `_edata` 纹丝不动，因为尾部被固定的目标文件占着。

### 7.8 状态机看门狗（"卡在开始聆听"）

- **层 1**：`kListening` 看门狗从进状态起无条件 15 s 计时，而实测一次 LLM 单调用 86 s ⇒ 用户还在说话时连接就被断了。改为 **45 s**。
- **层 2**：真正卡在 `kConnecting`（健康 0.32~1.3 s，异常 15.9~31.7 s），而 UI 把 `kConnecting` 映射为"正在聆听"**谎报可以说话**，麦克风要真进 `kListening` 才开 ⇒ 用户的话全丢。新增 **20 s 建连超时**退回 standby。

> 另有一条易错点：一次物理按键会在 UI 侧产生 2~3 次 `ask: "[BOOT] click"` 打印，**这是 UI 重复读同一个值，不是事件重复发生**。

---

## 八、已知问题与后续计划

1. **播报余量仍偏薄。** 倍率中位仅 ≈1.0（750 ms reserve ÷ (1−0.86) ≈ 5.4 s 持续劣势即耗尽）。若主观听测仍卡，下一步把 `V1000B_MIN_RESERVE` 由 750 ms 提到 1000 ms。
2. **首次 TLS 握手偶发 15.9 s**（对比正常 0.32~0.68 s，属网络/DNS 抖动），是当前用户可感知的最大延迟源。方向：开机空闲预热一条 WebSocket（需绕开每轮重建连接的策略）。
3. **静态 IP 有被 DHCP 续租冲掉的理论风险**（实测 21 s 时仍是 `.154`，暂未复现）。彻底做法是在路由器上给 MAC `90:70:69:16:2A:3C` 做静态 DHCP 绑定；备选退路是通过 MTP 改写 K230 SD 卡上的 `VOX3_IP`。
4. **K230 侧参数未调**：`SMOOTH_ALPHA 0.5→0.75`、`STABLE_FRAMES 10→4`、`SEND_HZ 10→15` 可进一步降低端到端迟滞（SD 卡经 MTP 可写）。
5. **RX 中断与 WiFi 闭源 FIQ 的并发冲突尚未根治**（当前用编译期开关 `CAPTURE_PROBE` 规避，置 1 可复现）；下一步尝试在 RX 中断期间临界区屏蔽 WiFi FIQ 验证假设。
6. **唤醒词尚未接入**，当前为按键 PTT；barge-in（播报中打断）未实现。
7. 已知内存泄漏约 2.6 KB/轮；未做系统级低功耗优化。
8. 米家设备控制凭据需用户自行配置（未内置）。

---

## 九、许可

Apache-2.0（与 openvela 上游一致）。
