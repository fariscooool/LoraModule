/**
 * @file app_lora_net.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 组网层(多对多):唯一识别码 + 信道分组 + 定点传输 + 帧协议 v2 + ACK/重传
 * @version 0.2
 * @date 2026-09-30
 *
 * @note 本模块解决三个问题:
 *       1) "谁是谁" —— 唯一识别码 = 模块地址(ADDH/ADDL) + 帧内源地址 SRC,
 *          开机时从模块寄存器读回作为唯一真源,掉电不丢。
 *       2) "串频"   —— 四层过滤:组号(地址高字节,硬件) / 信道(硬件) /
 *          模块地址精确匹配(硬件定点传输) / 帧内组号(软件)。
 *          ★ 批量生产的关键: 组号必须进地址高字节, 见下方"地址空间划分"。
 *       3) "主机要确认" —— 上行帧等 ACK,超时随机退避重传;上行后开接收窗口。
 *
 * @note 硬件前提(已按 E32/SX1278 系列编写,寄存器位序以你的模块手册为准):
 *       - REG0(0x02): [7:5]空速 [4:3]校验 [2:0]波特率
 *       - REG1(0x03): 信道 (E32-433 系列 0x00~0x1F, 410+CH MHz)
 *       - REG2(0x04): [7]定点传输 [6]IO驱动 [5:3]唤醒时间 [2]FEC [1:0]发射功率
 *       - 定点传输: 发送前在数据前加 3 字节 [目标ADDH][目标ADDL][目标信道]
 *
 * @note 接入方式(App/app_main.c):
 *       app_init():  app_lora_init() 之后 -> app_lora_net_init();
 *                    app_lora_net_set_signal(&s_sig); app_lora_net_set_power(&s_pwr);
 *       app_task():  信号跳变 -> app_lora_net_uplink_signal(state);   // 替换 app_lora_signal()
 *                    每轮循环 -> app_lora_net_task();
 *                    进 Stop 前 -> if (app_lora_net_busy() == 0U) { ...进入低功耗... }
 *       构建: 需把 App/app_lora_net.c 加入 CMakeLists.txt 的 target_sources。
 */
#ifndef APP_LORA_NET_H
#define APP_LORA_NET_H

#include <stdint.h>

#include "app_config.h"
#include "app_lora.h"

/*==============================================================================
 * 一、组网参数(默认值集中在这里)
 *   如果以后想统一收口到 app_config.h,直接把这些 #define 拷过去即可,
 *   本文件用的是 #ifndef 保护,不会重复定义。
 *============================================================================*/

/*------------------------------------------------------------------------------
 * 多套设备共存的地址空间划分(批量生产核心约定)
 *
 *   16bit 模块地址 = [组号 8bit][节点序号 8bit]
 *     组号     : 一套设备(1 主机 + 3 从机)占一个组号, 同空域内不重复
 *     节点序号 : 0x00 = 主机, 0x01~0xFE = 从机编号
 *
 *   例: 组号 0x12 -> 主机 0x1200 / 从机 0x1201 0x1202 0x1203
 *
 *   为什么组号要放进"地址"而不是只靠帧内 GRP 字段:
 *     定点传输下 E32 按"目标地址 == 自身地址"做硬件过滤, 地址不匹配的帧
 *     一个字节都不会吐给串口 —— 别套设备的流量既不占本机接收窗口, 也不会
 *     被误执行。帧内 GRP 只是第三道软件保险(挡"地址配错/人手工配错")。
 *
 *   !! 反例(批量生产的经典事故): 所有套的主机地址都用 0x0000, 所有从机都用
 *      0x0001~0x0003 —— 同信道下 A 套从机的上报会被 A/B/C… 所有主机收到。
 *
 *   容量: 256 组 × 每组 255 节点, 不受 E32 只有 32 个信道的限制。
 *---------------------------------------------------------------------------*/

/* 组号:一套设备(1 主机 + N 从机)共用一个组号
 * !! 批量生产时每套必须不同 !! 0 = 不校验组号(兼容单组老部署) */
#ifndef APP_LORA_NET_GROUP_ID
#define APP_LORA_NET_GROUP_ID       0x01U
#endif

/* 本机在组内的节点序号:0x00=主机, 0x01~0xFE=从机 */
#ifndef APP_LORA_NET_NODE_INDEX
#define APP_LORA_NET_NODE_INDEX     ((uint8_t)APP_DEVICE_ADDR)
#endif

/* 1 = 地址由 [组号][节点序号] 合成(推荐; 批量生产只需改 组号 一个宏)
 * 0 = 地址直接用下面的 APP_LORA_NET_NODE_ID(单套/自定义寻址) */
#ifndef APP_LORA_NET_ADDR_FROM_GROUP
#define APP_LORA_NET_ADDR_FROM_GROUP 1U
#endif

/* 本机节点号(唯一识别码的低 16 位)
 * 说明:只作为"模块读不到配置时"的兜底值。正常运行以模块寄存器里的地址为准。 */
#ifndef APP_LORA_NET_NODE_ID
#if (APP_LORA_NET_ADDR_FROM_GROUP == 1U)
#define APP_LORA_NET_NODE_ID        ((uint16_t)(((uint16_t)APP_LORA_NET_GROUP_ID << 8) | \
                                                 (uint16_t)APP_LORA_NET_NODE_INDEX))
#else
#define APP_LORA_NET_NODE_ID        ((uint16_t)APP_DEVICE_ADDR)
#endif
#endif

/* 主机地址(上行帧的目标地址,定点传输用) = 同组 + 节点序号 0x00
 * !! 绝不能全网都用 0x0000: 那样同信道所有主机都会收到同一台从机的上报 !! */
#ifndef APP_LORA_NET_HOST_ADDR
#if (APP_LORA_NET_ADDR_FROM_GROUP == 1U)
#define APP_LORA_NET_HOST_ADDR      ((uint16_t)((uint16_t)APP_LORA_NET_GROUP_ID << 8))
#else
#define APP_LORA_NET_HOST_ADDR      0x0000U
#endif
#endif

/* 广播地址(定点传输的 0xFFFF): 同信道所有设备都会被唤醒, 慎用 */
#ifndef APP_LORA_NET_BROADCAST_ADDR
#define APP_LORA_NET_BROADCAST_ADDR 0xFFFFU
#endif

/* 信道(E32-433 系列只有 0x00~0x1F,对应 410+CH MHz)
 * !! 必须与同组所有设备(含主机)一致,否则物理层就不通 !!
 * 注意: 地址只解决"收错人", 不解决"空口碰撞"。同空域内多套设备应尽量
 *       分到不同信道, 规划方法见 Docs/LoRa_Protocol_v2.md "多套设备共存"。 */
#ifndef APP_LORA_NET_CHANNEL
#define APP_LORA_NET_CHANNEL        0x0AU
#endif
#ifndef APP_LORA_NET_CHANNEL_MAX
#define APP_LORA_NET_CHANNEL_MAX    0x1FU
#endif

/* 信道分配策略(同一套设备内所有节点必须取值一致, 主机侧也要一致):
 *   0 = 固定用 APP_LORA_NET_CHANNEL (缺省, 兼容现有主机/单套部署)
 *   1 = 信道 = 组号 % 信道总数 (按组散频; 批量生产推荐, 减轻空口碰撞) */
#ifndef APP_LORA_NET_CHANNEL_FROM_GROUP
#define APP_LORA_NET_CHANNEL_FROM_GROUP 0U
#endif
/* 信道 0 对应的频率(MHz);不同频段型号不同,按手册改 */
#ifndef APP_LORA_NET_CHANNEL_BASE_MHZ
#define APP_LORA_NET_CHANNEL_BASE_MHZ   410U
#endif

/* 期望的模块参数(自检/写入用) */
#ifndef APP_LORA_NET_AIR_RATE
#define APP_LORA_NET_AIR_RATE       AIR_9_6K     /* 空速: 高一点缩短空中时间, 降冲突 */
#endif
#ifndef APP_LORA_NET_BAUD
#define APP_LORA_NET_BAUD           BAUD_9600    /* 串口波特率(与 MCU 一致) */
#endif
#ifndef APP_LORA_NET_PARITY

#define APP_LORA_NET_PARITY         PARITY_8N1
#endif
#ifndef APP_LORA_NET_WAKEUP_TIME
#define APP_LORA_NET_WAKEUP_TIME    WAKEUP_250MS /* 不用 WOR, 该值无实际影响 */
#endif
#ifndef APP_LORA_NET_TX_POWER
#define APP_LORA_NET_TX_POWER       0x00U        /* [1:0] 0=最大, 按现场需求调小 */
#endif
#ifndef APP_LORA_NET_FEC_ENABLE
#define APP_LORA_NET_FEC_ENABLE     1U           /* 短帧建议开 FEC */
#endif

/*==============================================================================
 * 二、时序与重传参数
 *============================================================================*/

/* 发送前随机退避上限(ms):同组多节点同时被唤醒时错开,避免同频相撞 */
#ifndef APP_LORA_NET_BACKOFF_MS
#define APP_LORA_NET_BACKOFF_MS     200U
#endif

/* 等 ACK 超时(ms):也就是"上行之后的接收窗口"长度 */
#ifndef APP_LORA_NET_ACK_TIMEOUT_MS
#define APP_LORA_NET_ACK_TIMEOUT_MS 500U
#endif

/* 重传次数上限(不含首次发送) */
#ifndef APP_LORA_NET_RETRY_MAX
#define APP_LORA_NET_RETRY_MAX      2U
#endif

/* 交付成功后的尾巴窗口(ms):给主机机会紧跟一条下行命令;0 = 立即休眠 */
#ifndef APP_LORA_NET_TAIL_MS
#define APP_LORA_NET_TAIL_MS        200U
#endif

/* 上电后的接收窗口(ms):方便上位机在现场下发"分配节点号/读配置"等命令 */
#ifndef APP_LORA_NET_BOOT_RX_WINDOW_MS
#define APP_LORA_NET_BOOT_RX_WINDOW_MS  2000U
#endif

/* 模式切换(M0/M1)后等模块稳定的时间(ms) */
#ifndef APP_LORA_NET_MODE_SETTLE_MS
#define APP_LORA_NET_MODE_SETTLE_MS 5U
#endif

/* 帧内字节间超时(ms):超时则丢弃半帧 */
#ifndef APP_LORA_NET_FRAME_GAP_MS
#define APP_LORA_NET_FRAME_GAP_MS   50U
#endif

/* 哪些上行帧需要 ACK(1=要, 0=不要)
 * 建议:信号帧要(重要), 电量/心跳不要(省功耗、减冲突) */
#ifndef APP_LORA_NET_ACK_FOR_SIGNAL
#define APP_LORA_NET_ACK_FOR_SIGNAL 1U
#endif
#ifndef APP_LORA_NET_ACK_FOR_POWER
#define APP_LORA_NET_ACK_FOR_POWER  0U
#endif

/* 是否允许主机远程改本机节点号(会写模块掉电保存寄存器, 默认关闭) */
#ifndef APP_LORA_NET_ALLOW_REMOTE_SET_NODE
#define APP_LORA_NET_ALLOW_REMOTE_SET_NODE  0U
#endif

/*==============================================================================
 * 三、帧协议 v2 常量(详见 Docs/LoRa_Protocol_v2.md)
 *   [0]     0x5A
 *   [1]     LEN   整帧字节数
 *   [2]     VER   0x02
 *   [3]     FUN   功能码 (bit7=1 表示应答/ACK)
 *   [4]     SRC_L 源节点号低字节
 *   [5]     SRC_H 源节点号高字节
 *   [6]     GRP   组号
 *   [7]     SEQ   本帧发送序号
 *   [8..]   DATA  DATA[0]=CORR(本帧所应答的对端 SEQ, 主动帧填 0), DATA[1..]=负载
 *   [LEN-1] CRC8
 *============================================================================*/
#define APP_LORA_NET_FRAME_HEAD     0x5AU
#define APP_LORA_NET_FRAME_VER      0x02U

/* 上行功能码(设备 -> 主机) */
#define APP_LORA_NET_FUN_HB         0x00U   /* 心跳, 负载空 */
#define APP_LORA_NET_FUN_SIGNAL     0x01U   /* 三相状态, 负载 1B */
#define APP_LORA_NET_FUN_POWER      0x02U   /* 电池电量, 负载 1B(%) */
#define APP_LORA_NET_FUN_CFG        0x03U   /* 模块配置回读, 负载 5B */

/* 下行功能码(主机 -> 设备) */
#define APP_LORA_NET_FUN_GET_POWER  0x10U   /* 取电量, 负载空 */
#define APP_LORA_NET_FUN_GET_SIGNAL 0x11U   /* 取信号, 负载空 */
#define APP_LORA_NET_FUN_SET_NODE   0x12U   /* 分配节点号, 负载 2B(小端) */
#define APP_LORA_NET_FUN_QUERY_CFG  0x13U   /* 读模块配置, 负载空 */

/* ACK 结果码(ACK 帧 DATA[1]) */
#define APP_LORA_NET_CODE_OK          0x00U
#define APP_LORA_NET_CODE_UNSUPPORTED 0x01U
#define APP_LORA_NET_CODE_BAD_PARAM   0x02U
#define APP_LORA_NET_CODE_BUSY        0x03U
#define APP_LORA_NET_CODE_DENIED      0x04U

/*==============================================================================
 * 四、类型与接口
 *============================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief E32 模块工作参数(逻辑值, 与寄存器位序解耦)
 */
typedef struct
{
    uint8_t addr_h;             /* 地址高字节(节点号 >8 位部分) */
    uint8_t addr_l;             /* 地址低字节                  */
    uint8_t air_rate;           /* 空速 0~7, 见 air_rate_t     */
    uint8_t parity;             /* 校验 0~3, 见 parity_t       */
    uint8_t baud;               /* 串口波特率 0~7, 见 ttl_baud_t */
    uint8_t channel;            /* 信道                       */
    uint8_t wakeup_time;        /* WOR 唤醒时间 0~7           */
    uint8_t io_drv_open_drain;  /* 1=开漏输出                 */
    uint8_t fec_enable;         /* 1=开 FEC                   */
    uint8_t tx_power;           /* 发射功率档位 0~3           */
    uint8_t fixed_point;        /* 1=定点传输(组网必须开)     */
} app_lora_e32_cfg_t;

/**
 * @brief 发送结果回调:ACK 收到或重传耗尽时调用
 * @param fun 上行功能码(APP_LORA_NET_FUN_*)
 * @param ok  1=主机已确认 0=失败(重传耗尽)
 */
typedef void (*app_lora_net_result_cb_t)(uint8_t fun, uint8_t ok);

/**
 * @brief 运行统计(现场排查用)
 */
typedef struct
{
    uint32_t tx_frames;    /* 实际发出的帧数(含重传) */
    uint32_t tx_retry;     /* 重传次数               */
    uint32_t tx_fail;      /* 最终失败的帧数         */
    uint32_t ack_ok;       /* 成功收到 ACK 的次数    */
    uint32_t ack_timeout;  /* 等 ACK 超时次数        */
    uint32_t rx_frames;    /* 收到并 CRC 通过的帧数  */
    uint32_t rx_crc_err;   /* CRC 错帧数             */
    uint32_t rx_filtered;  /* 被地址/组号过滤掉的帧数 */
    uint32_t cmd_recv;     /* 收到的下行命令数       */
} app_lora_net_stats_t;

/*-------------- 初始化与配置 ----------------*/

/**
 * @brief 组网层初始化
 * @note  在 app_lora_init() 之后调用:
 *        1) 读模块寄存器 -> 节点号/信道以模块为准(唯一识别码真源)
 *        2) 与期望参数不一致时打印告警
 *        3) 上电开 APP_LORA_NET_BOOT_RX_WINDOW_MS 接收窗口, 之后休眠
 */
void app_lora_net_init(void);

/**
 * @brief 组装"期望的模块参数"(节点号/信道来自上面的宏)
 */
void app_lora_net_cfg_defaults(app_lora_e32_cfg_t *cfg);

/**
 * @brief 读回模块寄存器原始 5 字节(ADDH ADDL SPED CHAN OPTION)
 * @retval 1=成功 0=失败
 * @note  会临时进出配置模式(切 M0/M1),退出后模块停在正常模式
 */
uint8_t app_lora_net_cfg_read_raw(uint8_t raw[5]);

/**
 * @brief 把期望参数写进模块(带写回显 + 读回比对)
 * @retval 1=成功 0=失败
 */
uint8_t app_lora_net_cfg_apply(const app_lora_e32_cfg_t *cfg);

/**
 * @brief 配置自检:读回 -> 按 E32 位序解码 -> 打印 -> 与期望比对(不一致则尝试写入)
 * @retval 1=最终一致 0=失败
 */
uint8_t app_lora_net_cfg_selftest(void);

#if (APP_DEBUG_ENABLE == 1)
/**
 * @brief 打印一份配置(解码后的可读形式)
 * @note  仅在 APP_DEBUG_ENABLE=1 时存在
 */
void app_lora_net_dump_cfg(const app_lora_e32_cfg_t *cfg);
#endif

/**
 * @brief 改本机节点号:写模块地址寄存器(掉电保存) + 更新内存中的识别码
 * @param  node_id 完整 16bit 地址 = [组号][节点序号];
 *                 调这个接口会同时改组号(高字节)
 * @retval 1=成功 0=失败
 */
uint8_t app_lora_net_set_node_id(uint16_t node_id);

/**
 * @brief 分配 组号 + 节点序号(批量生产/上位机下发用)
 * @param  group      组号(1~0xFE)
 * @param  node_index 节点序号(0x01~0xFE; 0x00 保留给主机)
 * @retval 1=成功 0=失败
 * @note  会写模块地址寄存器(掉电保存); 若开了
 *        APP_LORA_NET_CHANNEL_FROM_GROUP 会同时按组号重写信道
 */
uint8_t app_lora_net_set_group(uint8_t group, uint8_t node_index);

/**
 * @brief 取本机节点号(唯一识别码) = [组号][节点序号]
 */
uint16_t app_lora_net_node_id(void);

/**
 * @brief 取本机组的上行目标地址(= 组号 << 8, 即主机地址)
 */
uint16_t app_lora_net_host_addr(void);

/**
 * @brief 取本机所属组号(批量生产时用于核对是否装错套)
 */
uint8_t app_lora_net_group(void);

/**
 * @brief 取本机当前信道(定点包头第 3 字节)
 */
uint8_t app_lora_net_channel(void);

/**
 * @brief 取芯片 96bit UID 派生的 32bit 短标识(用于入网登记/日志)
 */
uint32_t app_lora_net_uid32(void);

/*-------------- 状态与数据 ----------------*/

/**
 * @brief 注册"当前三相状态"的取值指针(可为 NULL, 则用最近一次上报的缓存值)
 */
void app_lora_net_set_signal(uint8_t *signal);

/**
 * @brief 注册"当前电量"的取值指针(可为 NULL, 则用最近一次上报的缓存值)
 */
void app_lora_net_set_power(uint8_t *power);

/**
 * @brief 注册发送结果回调(可为 NULL)
 */
void app_lora_net_set_result_cb(app_lora_net_result_cb_t cb);

/**
 * @brief 上报三相状态(数据域 1 字节, 按 APP_LORA_NET_ACK_FOR_SIGNAL 决定是否等 ACK)
 * @retval 1=已入队 0=参数非法
 * @note  非阻塞:立即返回, 实际发送在 app_lora_net_task() 里按随机退避发出
 */
uint8_t app_lora_net_uplink_signal(uint8_t state);

/**
 * @brief 上报电池电量
 */
uint8_t app_lora_net_uplink_power(uint8_t percent);

/**
 * @brief 上报心跳(负载空)
 * @note  多机组网下不建议周期调用, 会平白增加冲突概率
 */
uint8_t app_lora_net_uplink_heartbeat(void);

/*-------------- 周期处理与窗口 ----------------*/

/**
 * @brief 组网层周期处理(主循环里调用)
 * @note  非阻塞:拆包 / 状态机推进 / 窗口超时关窗休眠 都在这里完成
 */
void app_lora_net_task(void);

/**
 * @brief 是否还有事情没做完(收发中/接收窗口未关)
 * @retval 1=别进 Stop  0=可以进 Stop
 */
uint8_t app_lora_net_busy(void);

/**
 * @brief 主动开一个接收窗口(模块保持正常模式 ms 毫秒, 期间可收下行命令)
 * @note  用于现场调试/上位机下发命令
 */
void app_lora_net_open_rx_window(uint32_t ms);

/**
 * @brief 取运行统计
 */
const app_lora_net_stats_t *app_lora_net_get_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LORA_NET_H */
