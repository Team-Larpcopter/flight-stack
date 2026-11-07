#!/usr/bin/env bash
# Download NVIDIA's official C-Fast-FoundationStereo dynamic-shape ONNX model
# and compile a TensorRT 10.x FP16 engine on the Jetson Orin Nano.
#
# Usage (inside container or via neural_depth_ws.sh build-engine [HEIGHT] [WIDTH]):
#   ./build_trt_engine.sh [HEIGHT] [WIDTH] [MODELS_DIR]
set -euo pipefail

H="${1:-320}"
W="${2:-480}"
MODELS_DIR="${3:-/models}"
ONNX_URL="https://huggingface.co/nvidia/c-fast-foundationstereo/resolve/main/onnx/c_fast_foundationstereo_bp2_dynbatch_dynhw.onnx"
ONNX_FILE="$MODELS_DIR/ffs_${H}x${W}_d160_i4.onnx"
if [[ ! -s "$ONNX_FILE" ]]; then
  ONNX_FILE="$MODELS_DIR/c_fast_foundationstereo_dynhw.onnx"
fi
ENGINE_FILE="$MODELS_DIR/fast_foundationstereo_${H}x${W}_fp16.engine"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

mkdir -p "$MODELS_DIR"

if [[ ! -s "$ONNX_FILE" ]]; then
  echo "Downloading C-Fast-FoundationStereo ONNX (~98 MB) to $ONNX_FILE..."
  curl -L --fail "$ONNX_URL" -o "$ONNX_FILE"
fi

echo "Building TensorRT FP16 engine for ${H}x${W} -> $ENGINE_FILE (Orin Nano 8GB profile)..."
"$TRTEXEC" \
  --onnx="$ONNX_FILE" \
  --saveEngine="$ENGINE_FILE" \
  --fp16 \
  --builderOptimizationLevel=1 \
  --maxAuxStreams=0 \
  --tacticSources=-CUDNN,-CUBLAS,-CUBLAS_LT \
  --skipInference \
  --memPoolSize=workspace:1024M

echo "Done! Engine saved at $ENGINE_FILE"
