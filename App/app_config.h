/**
 * @file app_config.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  设备级配置集中管理(应用层全局配置头)
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   所有“这块板子/这个产品”相关的可配置项都集中在这里,
 *         用宏定义,方便不同硬件/不同客户需求时只改这一个文件。
 *         本文件应被包含在其它 App 源文件的最前面。
 *
 * @warning 正式发布固件前,请把 APP_DEBUG_ENABLE 置 0 以屏蔽所有调试输出/命令。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*==============================================================================
 * 一、设备基本信息
 *============================================================================*/

#define APP_DEVICE_NAME         "ABCPhase-LP"        /* 设备/产品名          */
#define APP_DEVICE_MODEL        "STM32L031G6U6"      /* MCU 型号(硬件平台)   */
#define APP_FW_VERSION          "1.0.0"              /* 固件版本             */
#define APP_DEVICE_ADDR         0x02                 /* 设备地址(可用于组网/
                                                        上位机识别)         */
#define APP_LORA_FREQ_CH        0x00                 /* LoRa 信道/频点索引,
                                                        具体含义按你的模块定 */
/*==============================================================================
 * 二、目标主机相关参数
 *============================================================================*/

/* 主机地址/信道:
 * 注:帧内地址字节度已统一为“设备地址”语义(上行=源设备地址,下行=目标设备地址,
 *     见“四、上报帧协议”);当前透明广播方案下本宏不参与收发过滤,保留给定点传输/
 *     后续组网(v2)使用。主机模块自身地址设为 0xFFFF(广播)不影响该过滤逻辑。 */
#define APP_HOST_ADDR           0x01                 /* 目标主机地址(暂作记录) */
#define APP_HOST_LORA_FREQ_CH   0x00                 /* 目标主机 LoRa 信道/频点索引 */

/*==============================================================================
 * 三、外部信号通道信息
 *  说明:通道编号与 Bsp 层一致(CH0=PB0, CH1=PB1, CH2=PB3)
 *============================================================================*/

#define APP_SIG_CH_COUNT        3                    /* 外部信号通道个数 */

#define APP_SIG_CH0_NAME        "CH0-PB0"            /* 通道显示名(供命令打印) */
#define APP_SIG_CH1_NAME        "CH1-PB1"
#define APP_SIG_CH2_NAME        "CH2-PB3"

/* 外部信号消抖时间(ms):电平需稳定这么久才认为消抖完成 */
#define APP_SIG_DEBOUNCE_MS     20U

/* 消抖时允许的最长等待(ms):
 * 电平一直在抖时最多阻塞这么久就放弃,避免主循环被拖住/拖长工作时间。
 * 一般取 APP_SIG_DEBOUNCE_MS 的若干倍即可。 */
#define APP_SIG_DEBOUNCE_MAX_MS 200U

/**
 * 挂接状态
 * 成功 0xAA
 * 失败 0x55
 */
#define APP_SIG_HOOKED_OK      0xAA
#define APP_SIG_HOOKED_FAIL    0x55


/*==============================================================================
 * 四、上报帧协议以及相关参数(v1.5:带 SEQ 与 ACK)
 *
 * 地址字节 [1] 的语义(两个方向都是“设备地址”):
 *   上行(设备->主机):[1] = 本机(源)地址,主机靠它区分哪台设备上报;
 *   下行(主机->设备):[1] = 目标设备地址,设备靠它判断这条 ACK/命令是不是给自己的。
 *   主机模块自己可以设为 0xFFFF(广播),但“回复”必须带具体设备地址 ——
 *   多台设备同信道时才不会把别人的 ACK 当成自己的。
 *   设备地址必须唯一;0xFF 保留为广播地址,不可用作设备地址。
 *
 * 上行帧(设备 -> 主机,信号帧/电量帧各一帧,背靠背发出):
 *   [0] 0x5A  [1] 本机地址  [2] LEN=7(整帧字节数,含帧头与 CRC)
 *   [3] FUN   [4] SEQ(每发一帧 +1,重发换新)  [5] 数据域  [6] CRC8(对 [0..5])
 *
 * 下行帧(主机 -> 设备;ACK 与点名命令同布局):
 *   [0] 0x5A  [1] 目标设备地址  [2] LEN=7(命令无数据域时为 5)
 *   [3] FUN|0x80(ACK)/FUN(命令)  [4] CORR 或命令SEQ  [5] CODE 或数据  [6] CRC8
 *   ACK:[4]=CORR(被确认帧的 SEQ),[5]=CODE(0=OK)
 *============================================================================*/

#define LORA_FRAME_HEAD     0x5A
#define LORA_FRAME_UPLINK_LEN 7U    /* 上行帧总长:头+地址+长度+FUN+SEQ+数据+CRC */
#define LORA_FRAME_ACK_LEN  7U      /* ACK 帧总长:头+地址+长度+FUN+CORR+CODE+CRC */
#define LORA_FRAME_MIN_LEN  5U      /* 头+地址+长度+FUN+CRC(数据域为空) */
#define LORA_FRAME_DATA_MAX 12U     /* 数据域上限 */
#define LORA_FRAME_BUF_MAX  (LORA_FRAME_MIN_LEN + LORA_FRAME_DATA_MAX)
#define LORA_FRAME_GAP_MS   50U     /* 字节间超时:超时则丢弃半帧 */

/* 上行(设备 -> 主机)功能码定义(可扩展) */
#define LORA_FUN_SIGNAL 0x01        // 信号帧，主要是ABC相开关状态
#define LORA_FUN_POWER 0x02         // 电池电量帧

/* 下行(主机 -> 设备)功能码:控制参数(需要时启用)
 * 注意:0x81/0x82 已让给 ACK 码(FUN|0x80),点名命令迁到 0x1x 区(与 v2 文档一致) */
#define LORA_FUN_GET_POWER 0x10        // 获取电池电量帧
#define LORA_FUN_GET_SIGNAL 0x11       // 获取信号帧

/* 上行确认(ACK)与重试参数 */
#define LORA_FRAME_ACK_BIT  0x80U   /* ACK 功能码 = 原 FUN | 该位 */
#define LORA_ADDR_BROADCAST 0xFFU   /* 下行帧广播地址:命令可广播,ACK 不接受广播 */

#define LORA_ACK_CODE_OK    0x00U   /* 0=主机已正确接收;非 0 均为失败(主机侧定义) */

/* 上行交付模式总开关(ACK/重试/退避相关代码全部保留,只做编译期裁剪):
 *   1 = 完整交付:两帧连发 + 单窗口收双 ACK + 缺帧重发 + 退避(见 Docs/LoRa_Protocol_v1.5.md §6)
 *   0 = 简化版(样机):两帧背靠背发出即结束 —— 不等 ACK、不重发、不占事务,
 *       发完可立即进 Stop;主机仍按协议回 ACK,设备直接忽略,空口帧格式完全不变 */
#define APP_LORA_ACK_ENABLE       0

#define APP_LORA_ACK_TIMEOUT_MS   300U  /* 等 ACK 窗口(ms);空速 0.3k 时需加大 */
#define APP_LORA_RETRY_MAX        2U    /* 每帧最多重发次数(每帧总发送 1+N 次) */
#define APP_LORA_RETRY_BACKOFF_MS 50U   /* 重发前退避(ms),错开碰撞 */
#define APP_LORA_TX_BACKOFF_MS    0U    /* 首帧发送前退避(ms);多节点同信道建议 0~150 */

/* LoRa 配置命令 */
#define LORA_CFG_CMD_WRITE  0xC0U   /* 写寄存器(掉电保存);临时写可改 0xC2 */
#define LORA_CFG_CMD_READ   0xC1U   /* 读参数 */

/*=============================================================================
 * 五、LoRa 配置模式相关参数
 *============================================================================*/

/* 配置模式相关时间参数 */
#define LORA_CFG_SWITCH_MS  20U     /* 模式切换后等待模块稳定(ms) */
#define LORA_CFG_READY_MS   100U    /* 等 AUX 就绪的超时(ms) */
#define LORA_CFG_REPLY_MS   300U    /* 等配置回包的超时(ms) */

/* 单次配置读/写的最大数据字节数 */
#define APP_LORA_CFG_DATA_MAX   5U

/*==============================================================================
 * 六、调试 / 串口命令
 *============================================================================*/

/* 调试总开关:
 *   1 = 开启调试(默认,开发调试阶段)
 *   0 = 屏蔽所有调试打印(正式发布固件时置 0)
 * 为 0 时,dbg_printf/dbg_send 等调用会被编译成空操作,不占代码/不执行。 */
#define APP_DEBUG_ENABLE        1

/* 串口命令解析开关:
 * 默认跟随 APP_DEBUG_ENABLE;一般发布时两者一起关闭。
 * (命令执行结果依赖 dbg_printf 打印,若单独开启请同时保持打印开启) */
#define APP_DEBUG_CMD_ENABLE    APP_DEBUG_ENABLE

/* 串口命令是否回显输入(1=回显,便于终端观察) */
#define APP_CMD_ECHO            0

/* 低功耗运行开关:
 *   1 = 正常进入 Stop 低功耗(发布/实测电流用)
 *   0 = 调试模式:主循环不进低功耗,一直运行
 *       —— 便于用调试器连接/单步(避免芯片一上电就睡进 Stop 连不上)。 */
#define APP_LOWPOWER_ENABLE     1

/* 上电保持运行窗口(ms):
 * 仅在 APP_LOWPOWER_ENABLE=1 时生效 —— 上电后前 N 毫秒不进入 Stop,
 * 仍正常运行/喂狗/响应命令,便于上电后用调试器连接或“按复位”追赶。
 * 想尽快进低功耗省电可设为 0(发布版通常设 0 或 500)。 */
#define APP_BOOT_KEEP_RUN_MS    10000U

/*==============================================================================
 * 七、电池电压检测(硬件:ADC1_IN6 / PA6)
 *  下面这些宏必须在包含 bsp_adc_pwr.h 之前定义才生效
 *  (各 .c 都在最前面包含本文件,满足该顺序)。
 *  驱动接口见 Bsp/bsp_adc_pwr.h。
 *============================================================================*/

/* 分压比:Vbat = Vadc * NUM / DEN
 * !! 请按板上实际电阻填写 !!
 *    例:R1=100k(上臂)、R2=100k(下臂) -> 2/1;
 *        电池直连 PA6(无分压)        -> 1/1 */
#define BSP_ADC_PWR_DIV_NUM     2U
#define BSP_ADC_PWR_DIV_DEN     1U

/* 参考电压(= VDDA)标称值 mV:
 * 仅在 BSP_ADC_PWR_USE_VREFINT = 0 时使用 */
#define BSP_ADC_PWR_VREF_MV     3300U

/* 1 = 用内部 VREFINT 实测 VDDA 再换算(推荐,结果不受 VDD 漂移影响)
 * 0 = 直接用上面的标称值,快一点但误差取决于 VDD 精度 */
#define BSP_ADC_PWR_USE_VREFINT 1U

/* 每次测量采样次数 / 单次转换超时 ms */
#define BSP_ADC_PWR_SAMPLES     8U
#define BSP_ADC_PWR_TIMEOUT_MS  5U

/* 电量百分比映射区间与低压告警阈值 mV
 * !! 占位值(按 2.0V ~ 3.0V 电池组),请按实际电池规格修改 !!
 *    注:若 VDD 就是电池电压本身,Vbat 会等于 VDDA,阈值需按实际电池组设置 */
#define BSP_ADC_PWR_BAT_EMPTY_MV 2000U
#define BSP_ADC_PWR_BAT_FULL_MV  3000U
#define BSP_ADC_PWR_LOW_MV       2200U

#endif /* APP_CONFIG_H */
