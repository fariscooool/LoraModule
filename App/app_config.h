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
#define APP_DEVICE_ADDR         0x01                 /* 设备地址(可用于组网/
                                                        上位机识别)         */
#define APP_LORA_FREQ_CH        0x00                 /* LoRa 信道/频点索引,
                                                        具体含义按你的模块定 */

/*==============================================================================
 * 二、外部信号通道信息
 *  说明:通道编号与 Bsp 层一致(CH0=PB0, CH1=PB1, CH2=PB3)
 *============================================================================*/

#define APP_SIG_CH_COUNT        3                    /* 外部信号通道个数 */

#define APP_SIG_CH0_NAME        "CH0-PB0"            /* 通道显示名(供命令打印) */
#define APP_SIG_CH1_NAME        "CH1-PB1"
#define APP_SIG_CH2_NAME        "CH2-PB3"

/* 每路信号“出现低电平”时通过 UART1 下发给 LoRa 的“参数”
 * (占位示例值,按你的实际协议/含义修改;
 *  运行中可通过调试串口命令 set 临时修改,重启恢复为这里的默认值) */
#define APP_SIG_CH0_PARAM       0x01
#define APP_SIG_CH1_PARAM       0x02
#define APP_SIG_CH2_PARAM       0x03

/* 外部信号消抖时间(ms):电平需稳定这么久才认为消抖完成 */
#define APP_SIG_DEBOUNCE_MS     20U

/* 消抖时允许的最长等待(ms):
 * 电平一直在抖时最多阻塞这么久就放弃,避免主循环被拖住/拖长工作时间。
 * 一般取 APP_SIG_DEBOUNCE_MS 的若干倍即可。 */
#define APP_SIG_DEBOUNCE_MAX_MS 200U

/*==============================================================================
 * 三、上报帧协议(LoRa 透明传输,原样字节下发)
 *   [0] [1] 帧头    0xAA 0x55
 *   [2]      类型    0x10 = 外部信号上报
 *   [3]      通道    0/1/2
 *   [4]      参数    见上方 APP_SIG_CHx_PARAM
 *   [5]      校验    前 5 字节累加和(取低 8 位)
 *   [6]      帧尾    0x0D
 * 总长 7 字节。!! 请按你实际使用的 LoRa 模块/网关协议修改 !!
 *============================================================================*/

#define FRM_HEAD0               0xAA
#define FRM_HEAD1               0x55
#define FRM_TYPE_REPORT         0x10
#define FRM_TAIL                0x0D

/*==============================================================================
 * 四、调试 / 串口命令
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
 * 五、电池电压检测(硬件:ADC1_IN6 / PA6)
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
