#!/bin/bash

# PTP Daemon 安装和配置脚本（支持指定网卡）
# 用法: sudo ./setup_ptpd_slave.sh [网卡名]

# 检查是否以root权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 此脚本必须以root权限运行" >&2
    echo "请使用sudo执行: sudo $0 [网卡名]" >&2
    exit 1
fi

# 设置默认网卡（可通过参数覆盖）
INTERFACE=${1:-eth2}
SERVICE_NAME="ptpd-${INTERFACE}-slave.service"

# 1. 安装ptpd（若未安装）
echo "[1/5] 正在安装ptpd..."
if ! apt update &>/dev/null || ! apt install -y ptpd &>/dev/null; then
    echo "错误: ptpd安装失败" >&2
    exit 1
fi

# 2. 创建服务文件
echo "[2/5] 正在创建PTPd服务配置文件（网卡: ${INTERFACE}）..."
cat > /etc/systemd/system/${SERVICE_NAME} <<EOF
[Unit]
Description=PTP Daemon (IEEE 1588) for ${INTERFACE}
After=network.target
Conflicts=systemd-timesyncd.service

[Service]
Type=simple
ExecStartPre=/bin/sh -c "systemctl stop systemd-timesyncd && systemctl disable systemd-timesyncd"
ExecStart=/usr/sbin/ptpd -P -C -g -i ${INTERFACE}
Restart=always
RestartSec=5s
User=root

[Install]
WantedBy=multi-user.target
EOF

# 3. 重载Systemd配置
echo "[3/5] 正在重载systemd配置..."
if ! systemctl daemon-reload; then
    echo "错误: systemd配置重载失败" >&2
    exit 1
fi

# 4. 启用开机自启
echo "[4/5] 正在启用服务开机自启..."
if ! systemctl enable ${SERVICE_NAME}; then
    echo "错误: 启用服务失败" >&2
    exit 1
fi

# 5. 启动服务
echo "[5/5] 正在启动服务..."
if ! systemctl start ${SERVICE_NAME}; then
    echo "错误: 启动服务失败" >&2
    exit 1
fi

# 验证状态
echo -e "\n服务状态:"
systemctl status ${SERVICE_NAME} --no-pager

echo -e "\nPTPd服务配置完成!"
echo "网卡: ${INTERFACE}"
echo "服务名: ${SERVICE_NAME}"


