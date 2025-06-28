#!/bin/bash

# 配置参数
VERSION="2.0.0"
GITHUB_DIR="code"
BASE_URL="https://raw.githubusercontent.com/willy-hack/future-projects/refs/tags/${VERSION}/${GITHUB_DIR}"
CODE_DIR="$HOME/code"
SERVICE_FILE="/etc/systemd/system/start-code.service"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查并创建目录
create_directory() {
    echo -e "${YELLOW}[1/5] 创建代码目录...${NC}"
    mkdir -p "$CODE_DIR" || {
        echo -e "${RED}错误：无法创建目录 $CODE_DIR${NC}"
        exit 1
    }
}

# 下载文件函数
download_file() {
    local file_url="$1"
    local target_path="$2"
    local file_name=$(basename "$target_path")
    
    echo -e "${YELLOW}下载 $file_name...${NC}"
    if ! wget --tries=3 --timeout=30 -q "$file_url" -O "$target_path"; then
        echo -e "${RED}错误：下载 $file_name 失败${NC}"
        echo -e "请检查: $file_url"
        return 1
    fi
    
    # 验证文件内容不是HTML错误页面
    if grep -q "<html" "$target_path"; then
        echo -e "${RED}错误：下载到无效的HTML文件${NC}"
        return 1
    fi
    
    return 0
}

# 设置权限
set_permissions() {
    echo -e "${YELLOW}[3/5] 设置文件权限...${NC}"
    chmod +x "$CODE_DIR/start.sh" || {
        echo -e "${RED}错误：无法设置 start.sh 可执行权限${NC}"
        return 1
    }
    
    chmod +x "$CODE_DIR/start-code.py" || {
        echo -e "${RED}错误：无法设置 start-code.py 可执行权限${NC}"
        return 1
    }
    
    sudo chmod 644 "$SERVICE_FILE" || {
        echo -e "${RED}错误：无法设置服务文件权限${NC}"
        return 1
    }
    
    sudo chown -R "$USER:$USER" "$CODE_DIR" || {
        echo -e "${RED}错误：无法更改代码目录所有者${NC}"
        return 1
    }
}

# 主安装流程
main() {
    echo -e "${GREEN}=== 开始安装 start-code 服务 ===${NC}"
    
    # 1. 创建目录
    create_directory
    
    # 2. 下载文件
    echo -e "${YELLOW}[2/5] 下载必要文件...${NC}"
    download_file "$BASE_URL/start-code.service" "/tmp/start-code.service" || exit 1
    download_file "$BASE_URL/start.sh" "$CODE_DIR/start.sh" || exit 1
    download_file "$BASE_URL/start-code.py" "$CODE_DIR/start-code.py" || exit 1
    download_file "$BASE_URL/camra.py" "$CODE_DIR/camra.py" || exit 1
    download_file "$BASE_URL/function.py" "$CODE_DIR/function.py" || exit 1
    download_file "$BASE_URL/HSV_write.py" "$CODE_DIR/HSV_write.py" || exit 1
    download_file "$BASE_URL/jetson_nano_binarization.py" "$CODE_DIR/jetson_nano_binarization.py" || exit 1
    download_file "$BASE_URL/jetson_nano_main_final.py" "$CODE_DIR/jetson_nano_main_final.py" || exit 1
    download_file "$BASE_URL/jetson_nano_main.py" "$CODE_DIR/jetson_nano_main.py" || exit 1
    download_file "$BASE_URL/calibration_data.npz" "$CODE_DIR/calibration_data.npz" || exit 1
    download_file "$BASE_URL/hsv_values.pkl" "$CODE_DIR/hsv_values.pkl" || exit 1
    sed -i 's/\r$//' ~/code/start-code.py
    # 移动服务文件
    sudo mv "/tmp/start-code.service" "$SERVICE_FILE" || {
        echo -e "${RED}错误：无法移动服务文件到系统目录${NC}"
        exit 1
    }
    
    # 3. 设置权限
    set_permissions || exit 1
    
    # 4. 启用服务
    echo -e "${YELLOW}[4/5] 配置系统服务...${NC}"
    sudo systemctl daemon-reload || {
        echo -e "${RED}错误：daemon-reload 失败${NC}"
        exit 1
    }
    
    sudo systemctl enable start-code.service || {
        echo -e "${RED}错误：启用服务失败${NC}"
        exit 1
    }
    
    # 5. 启动服务
    echo -e "${YELLOW}[5/5] 启动服务...${NC}"
    sudo systemctl restart start-code.service || {
        echo -e "${RED}错误：启动服务失败${NC}"
        journalctl -u start-code.service -n 10 --no-pager
        exit 1
    }
    
    # 验证
    echo -e "${GREEN}\n=== 安装完成 ===${NC}"
    echo -e "服务状态："
    systemctl status start-code.service --no-pager -l
    
    echo -e "\n文件位置："
    echo -e "脚本文件: $CODE_DIR/start.sh"
    echo -e "Python文件: $CODE_DIR/start-code.py"
    echo -e "服务文件: $SERVICE_FILE"
}

# 执行主函数
main