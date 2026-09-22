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

/* 外部信号消抖时间(ms):低电平需稳定这么久才认为是有效事件 */
#define APP_SIG_DEBOUNCE_MS     20U

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

#endif /* APP_CONFIG_H */
