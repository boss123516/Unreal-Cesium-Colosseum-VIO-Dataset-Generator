#!/usr/bin/env bash
set -Eeuo pipefail

echo "===== OS ====="
grep -E '^(NAME|VERSION|VERSION_ID)=' /etc/os-release || true
uname -r

echo
echo "===== CPU ====="
lscpu | grep -E 'Model name|CPU\(s\)|Thread|Core|Socket' || true

echo
echo "===== MEMORY ====="
free -h

echo
echo "===== DISK ====="
df -h "$HOME"

echo
echo "===== NVIDIA ====="
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi                 --query-gpu=name,memory.total,driver_version                 --format=csv,noheader
else
    echo "nvidia-smi not found"
fi

echo
echo "===== VULKAN ====="
if command -v vulkaninfo >/dev/null 2>&1; then
    vulkaninfo --summary 2>/dev/null | head -n 80
else
    echo "vulkaninfo not found"
fi
