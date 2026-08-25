#ifndef RESOURCE_H
#define RESOURCE_H

/* 资源源 ID */
#define IDI_APP_ICON            100

/* tab 索引 */
#define TAB_CONFIG              0
#define TAB_UPGRADE             1
#define TAB_MODBUS              2
#define TAB_HISTORY             3
#define TAB_COUNT               4

/* ===== 通用控件 (9xxx) ===== */
#define IDC_STATUSBAR           9001

/* ===== tab1 控件 ID (1xxx) ===== */
#define IDC_CFG_DISCOVER_BTN    1001
#define IDC_CFG_DEVLIST         1002   /* 下拉框 (CBS_DROPDOWNLIST) */
#define IDC_CFG_CONNECT         1003   /* UDP 连接/断开 toggle 按钮 */
#define IDC_CFG_IP1             1010   /* 目标设备 IP 4 段 */
#define IDC_CFG_IP2             1011
#define IDC_CFG_IP3             1012
#define IDC_CFG_IP4             1013
#define IDC_CFG_GETVER          1014
#define IDC_CFG_REBOOT          1015
#define IDC_CFG_VERSION         1016   /* 静态文本, 显示版本 */
/* 网络参数 */
#define IDC_CFG_NIP1            1020   /* 新 IP 4 段 */
#define IDC_CFG_NIP2            1021
#define IDC_CFG_NIP3            1022
#define IDC_CFG_NIP4            1023
#define IDC_CFG_NIP_APPLY       1024
/* Modbus 参数 */
#define IDC_CFG_MB_SLAVE        1030
#define IDC_CFG_MB_BAUD         1031   /* 下拉 */
#define IDC_CFG_MB_APPLY        1032
#define IDC_CFG_MB_READ         1033
/* 时间设置 */
#define IDC_CFG_TIME_APPLY      1042   /* 应用本机时间到设备 */
#define IDC_CFG_TIMER           1070   /* SetTimer id (1s 刷新本机时间显示) */
/* 运维 */
#define IDC_CFG_FACTORY         1050
/* 日志 */
#define IDC_CFG_LOG             1060   /* 多行只读 EDIT */

/* ===== tab2 控件 ID (2xxx) — Task 7 固件升级 ===== */
#define IDC_UPG_CHAN_UDP        2001   /* 单选: UDP 通道 */
#define IDC_UPG_CHAN_CAN        2002   /* 单选: CAN 通道 */
#define IDC_UPG_IP1             2010   /* UDP 目标 IP 4 段 */
#define IDC_UPG_IP2             2011
#define IDC_UPG_IP3             2012
#define IDC_UPG_IP4             2013
#define IDC_UPG_TEST            2014   /* 测试模式 复选框 */
#define IDC_UPG_CAN_DEV         2020   /* PCAN 设备下拉 */
#define IDC_UPG_CAN_BAUD        2021   /* PCAN 波特率下拉 */
#define IDC_UPG_CAN_CONN        2022   /* 连接/断开 按钮 (UDP/CAN 通用) */
#define IDC_UPG_CAN_REFRESH     2023   /* PCAN 刷新设备 按钮 */
#define IDC_UPG_CAN_BOOT        2024   /* MCUboot 紧急救援模式 复选框 */
#define IDC_UPG_FILE            2030   /* 固件路径 EDIT */
#define IDC_UPG_BROWSE          2031   /* 浏览 按钮 */
#define IDC_UPG_FILEINFO        2032   /* 静态: magic/size/keyhash */
#define IDC_UPG_START           2040   /* 开始升级 按钮 */
#define IDC_UPG_CANCEL          2041   /* 取消 按钮 */
#define IDC_UPG_PROGRESS        2042   /* 进度条 */
#define IDC_UPG_STATUS          2043   /* 静态: 状态文字 */
#define IDC_UPG_LOG             2044   /* 多行日志 */
#define IDC_UPG_VERSION         2045   /* 静态: 设备版本号显示 */
#define IDC_UPG_GETVER          2046   /* 查询版本 按钮 */
#define IDC_UPG_REBOOT          2047   /* 重启设备 按钮 */

/* ===== tab3 控件 ID (3xxx) — Task 8 Modbus 调试 ===== */
#define IDC_MB_CHAN_TCP         3001   /* 单选: TCP 通道 */
#define IDC_MB_CHAN_RTU         3002   /* 单选: RTU 通道 */
#define IDC_MB_IP1              3010   /* TCP 目标 IP 4 段 */
#define IDC_MB_IP2              3011
#define IDC_MB_IP3              3012
#define IDC_MB_IP4              3013
#define IDC_MB_PORT             3014   /* TCP 端口 (默认 502) */
#define IDC_MB_COM              3020   /* 串口下拉 COM1-COM32 */
#define IDC_MB_BAUD             3021   /* 波特率下拉 */
#define IDC_MB_UID              3022   /* RTU 从机 ID */
#define IDC_MB_CONNECT          3030   /* 连接按钮 */
#define IDC_MB_DISCONNECT       3031   /* 断开按钮 */
#define IDC_MB_STATUS           3032   /* 状态灯 + 文字 */
#define IDC_MB_REFRESH_ALL      3040   /* 刷新全部按钮 */
#define IDC_MB_AUTOREF          3041   /* 自动刷新复选框 */
#define IDC_MB_AUTOREF_INT      3042   /* 自动刷新间隔 ms */
#define IDC_MB_REG_LIST         3050   /* 寄存器 ListView */
#define IDC_MB_REG_QUERY        3051   /* 查询选中行按钮 */
#define IDC_MB_LOG              3060   /* 多行只读 EDIT */
/* DI/DO/AI 动态创建子控件 (16+8+4 个), ID 自 3100 起 */
#define IDC_MB_DI_BASE          3100   /* DI1..DI16 = 3100..3115 */
#define IDC_MB_DO_BASE          3120   /* DO1..DO8 = 3120..3127 */
#define IDC_MB_AI_BASE          3130   /* AI1..AI4 = 3130..3133 */
#define IDC_MB_TIMER            1      /* SetTimer id */

/* ===== tab4 控件 ID (4xxx) — Task 历史记录解析 ===== */
#define IDC_HIST_OPEN           4001   /* 打开 .raw 历史文件 */
#define IDC_HIST_EXPORT         4002   /* 导出 CSV */
#define IDC_HIST_INFO           4003   /* 静态: 文件信息 */
#define IDC_HIST_LIST           4010   /* 记录 ListView */
#define IDC_HIST_LOG            4020   /* 多行只读 EDIT */

#endif /* RESOURCE_H */
