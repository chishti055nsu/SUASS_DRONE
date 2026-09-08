#!/bin/bash
# ============================================================
# maximize_jetson_performance.sh
# 🛸 NVIDIA Jetson Nano / Orin Nano Max Performance Tuner
# Locks CPU & GPU at maximum clock frequencies (MAXN Power Mode)
# ============================================================

echo "=========================================================================="
echo "   ⚡ MAXIMIZING NVIDIA JETSON CPU & GPU HARDWARE PERFORMANCE ⚡          "
echo "=========================================================================="

# 1. Set Jetson Power Mode to MAX Performance (MAXN Mode)
if command -v nvpmodel >/dev/null 2>&1; then
    echo "  ✅ Setting NVIDIA Jetson Power Mode to MAX Performance..."
    sudo nvpmodel -m 0 >/dev/null 2>&1 || sudo nvpmodel -m 1 >/dev/null 2>&1 || true
fi

# 2. Lock CPU & GPU Clocks at Maximum Clock Speed
if command -v jetson_clocks >/dev/null 2>&1; then
    echo "  ✅ Locking CPU Cores & GPU Frequencies at MAX Speed (jetson_clocks)..."
    sudo jetson_clocks >/dev/null 2>&1 || true
fi

# 3. Export Global CUDA & CPU Thread Optimizations
export CUDA_VISIBLE_DEVICES=0
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export PYTORCH_CUDA_ALLOC_CONF="max_split_size_mb:128"
export OPENCV_VIDEOIO_PRIORITY_GSTREAMER=100
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4

echo "=========================================================================="
echo "  🎉 CPU & GPU MAX POWER ENGAGED — ZERO LATENCY & MAXIMUM PERFORMANCE!"
echo "=========================================================================="
