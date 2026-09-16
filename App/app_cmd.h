/**
 * @file app_cmd.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  调试串口(UART2)命令解析模块接口
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   用于开发调试:不接外部信号也能手动触发/查看/修改本设备行为。
 *         发布固件时把 app_config.h 中 APP_DEBUG_ENABLE(或 APP_DEBUG_CMD_ENABLE)
 *         置 0 即可整段关闭本功能。
 */
#ifndef APP_CMD_H
#define APP_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 轮询处理调试串口命令(放到主循环里周期调用即可)
 * @note  内部按行读取 UART2 收到的字符并解析执行
 */
void app_cmd_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CMD_H */
