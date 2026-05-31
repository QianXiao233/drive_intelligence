#!/bin/bash
# SmartCarUI Linux 一键编译脚本
# 用法: bash build.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================="
echo " SmartCarUI - Linux Build"
echo "=============================="

# 1. qmake 生成 Makefile
echo ""
echo "[1/3] qmake ..."
qmake SmartCarUI.pro

# 2. 编译
echo ""
echo "[2/3] make -j$(nproc) ..."
make -j$(nproc)

# 3. 把 voices/ 复制到 exe 同目录
echo ""
echo "[3/3] 部署 voices/ ..."
BUILD_DIR="$SCRIPT_DIR"
# 如果有 build/ 子目录，可能 exe 在里面
if [ -d "build" ]; then
    # 查找 build 目录下的 SmartCarUI 可执行文件
    EXE_DIR=$(find build -name "SmartCarUI" -type f -exec dirname {} \; 2>/dev/null | head -1)
    if [ -n "$EXE_DIR" ]; then
        BUILD_DIR="$EXE_DIR"
    fi
fi

if [ -d "voices" ]; then
    mkdir -p "$BUILD_DIR/voices"
    cp -r voices/*.wav "$BUILD_DIR/voices/"
    echo "   voices/ -> $BUILD_DIR/voices/ ($(ls voices/*.wav 2>/dev/null | wc -l) 个文件)"
fi

echo ""
echo "=============================="
echo " 编译完成！"
echo " 可执行文件: $BUILD_DIR/SmartCarUI"
echo " 音频文件:   $BUILD_DIR/voices/"
echo "=============================="
echo ""
echo "运行: cd $BUILD_DIR && ./SmartCarUI"
