#!/bin/bash

# PTP Master 安装和配置脚本
# 用法: sudo ./setup_ptp_master.sh [网卡名] (默认: eth1)

# 检查是否以 root 权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 此脚本必须以 root 权限运行" >&2
    echo "请使用 sudo 执行: sudo $0 [网卡名]" >&2
    exit 1
fi

# 设置默认网卡名 (可修改或通过参数传递)
INTERFACE=${1:-eth1}
SERVICE_NAME="ptpd-${INTERFACE}-master.service"

# 1. 安装 ptpd（若未安装）
echo "[1/5] 正在安装 ptpd..."
if ! apt update &> /dev/null; then
    echo "错误: apt 更新失败" >&2
    exit 1
fi

if ! apt install -y ptpd &> /dev/null; then
    echo "错误: ptpd 安装失败" >&2
    exit 1
fi

# 2. 创建服务文件
echo "[2/5] 正在创建 PTP Master 服务配置文件 (接口: ${INTERFACE})..."
cat > /etc/systemd/system/${SERVICE_NAME} << EOF
[Unit]
Description=Precision Time Protocol (PTP) Daemon (Master on ${INTERFACE})
After=network.target

[Service]
Type=simple
ExecStart=/usr/sbin/ptpd -P -C -M -i ${INTERFACE}
Restart=always
RestartSec=5s
User=root

[Install]
WantedBy=multi-user.target
EOF

# 3. 重载 Systemd 配置
echo "[3/5] 正在重载 systemd 配置..."
if ! systemctl daemon-reload; then
    echo "错误: systemd 配置重载失败" >&2
    exit 1
fi

# 4. 启用并启动服务
echo "[4/5] 正在启用并启动 PTP Master 服务..."
if ! systemctl enable ${SERVICE_NAME}; then
    echo "错误: 启用服务失败" >&2
    exit 1
fi

if ! systemctl start ${SERVICE_NAME}; then
    echo "错误: 启动服务失败" >&2
    exit 1
fi

# 5. 显示服务状态
echo "[5/5] PTP Master 服务状态:"
systemctl status ${SERVICE_NAME} --no-pager

echo -e "\nPTP Master 配置完成!"
echo "服务名称: ${SERVICE_NAME}"
echo "监听接口: ${INTERFACE}"

