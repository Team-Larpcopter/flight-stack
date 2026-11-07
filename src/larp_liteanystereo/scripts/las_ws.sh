#!/usr/bin/env bash
# Lite Any Stereo V2 (LAS2) model pipeline for the drone.
#
#   las_ws.sh export [SIZE] [HEIGHT] [WIDTH] [MAX_DISP]   (any x86/ARM machine with docker;
#        run it on a laptop: needs CPU PyTorch + torchvision + timm, which the Jetson
#        image lacks) -> $LAS_OUT/las2_<size>_<H>x<W>_d<max_disp>.onnx
#   las_ws.sh engine [SIZE] [HEIGHT] [WIDTH] [MAX_DISP]   (Jetson HOST) TensorRT FP16
#        engine from that ONNX -> $MODELS_DIR/las2_<size>_<H>x<W>_d<max_disp>_fp16.engine
#   las_ws.sh build                                       (Jetson HOST) build this package
#        into the neural workspace (next to larp_neural_depth, whose node it runs)
#
# Defaults: SIZE m (s|m|l|h), 320 x 480 (both divisible by 32, as LAS requires),
# MAX_DISP 192 (0.12 m at 480 px wide with the OAK's 7.55 cm baseline).
# Copy the ONNX from the laptop to the Jetson's $MODELS_DIR before `engine`:
#   scp las_models/las2_m_320x480_d192.onnx dronehacks@<drone>:neural_depth_ws/models/
#
# Upstream: github.com/TomTomTommi/LiteAnyStereo (MIT) at a pinned commit; weights
# from huggingface.co/tomtomtommi/LiteAnyStereoV2.
set -euo pipefail
SIZE="${2:-m}"; H="${3:-320}"; W="${4:-480}"; D="${5:-192}"
LAS_COMMIT=8c97bd4c4da3712c2ac60003a23201dfdb5935f4
NAME="las2_${SIZE}_${H}x${W}_d${D}"
LAS_OUT="${LAS_OUT:-$PWD/las_models}"
NEURAL_WS="${NEURAL_WS:-$HOME/neural_depth_ws}"
MODELS_DIR="${MODELS_DIR:-$NEURAL_WS/models}"
INTEGRATION="${INTEGRATION:-$HOME/flight_integration_ws_20261107}"
IMAGE="${IMAGE:-larp-integration:20261107}"
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
[[ $SIZE =~ ^(s|m|l|h)$ ]] || { echo "SIZE must be s, m, l or h"; exit 2; }
(( H % 32 == 0 && W % 32 == 0 )) || { echo "HEIGHT and WIDTH must be divisible by 32"; exit 2; }

case "${1:-}" in
  export)
    mkdir -p "$LAS_OUT"
    CKPT="LAS2_${SIZE^^}.pth"
    # Throwaway CPU container, memory-capped. Upstream code is fetched at the pinned
    # commit; its committed __pycache__ is deleted before anything is imported.
    docker run --rm --memory=4g -v "$LAS_OUT:/out" python:3.10-slim bash -c "set -e
      apt-get update -qq && apt-get install -y -qq curl libgl1 libglib2.0-0 >/dev/null
      pip install -q --index-url https://download.pytorch.org/whl/cpu --extra-index-url https://pypi.org/simple torch torchvision
      pip install -q timm onnx scipy imageio pillow opencv-python-headless
      mkdir -p /las && cd /las
      curl -fsSL https://github.com/TomTomTommi/LiteAnyStereo/archive/$LAS_COMMIT.tar.gz | tar -xz --strip-components=1
      find . -name __pycache__ -type d -prune -exec rm -rf {} +
      [ -s /out/$CKPT ] || curl -fL -o /out/$CKPT https://huggingface.co/tomtomtommi/LiteAnyStereoV2/resolve/main/$CKPT
      python3 export_onnx.py --version las2 --model_size $SIZE --restore_ckpt /out/$CKPT \
        --width $W --height $H --max_disp $D --output_name /out/$NAME.onnx
      python3 - <<PY
import onnx
m = onnx.load('/out/$NAME.onnx'); onnx.checker.check_model(m)
print('inputs', [(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input])
print('outputs', [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
PY"
    ls -la "$LAS_OUT/$NAME.onnx"
    ;;
  engine)
    [[ -s "$MODELS_DIR/$NAME.onnx" ]] || { echo "missing $MODELS_DIR/$NAME.onnx (run export on a laptop and copy it)"; exit 1; }
    docker run --rm --runtime nvidia -v "$MODELS_DIR:/models" --entrypoint bash "$IMAGE" -c \
      "/usr/src/tensorrt/bin/trtexec --onnx=/models/$NAME.onnx --saveEngine=/models/${NAME}_fp16.engine \
        --fp16 --builderOptimizationLevel=3 --memPoolSize=workspace:2048M --skipInference"
    ls -la "$MODELS_DIR/${NAME}_fp16.engine"
    ;;
  build)
    mkdir -p "$NEURAL_WS/src"
    rm -rf "$NEURAL_WS/src/larp_liteanystereo"
    cp -r "$(dirname "$HERE")" "$NEURAL_WS/src/larp_liteanystereo"
    docker run --rm --memory=2g -v "$INTEGRATION:/integration:ro" -v "$NEURAL_WS:/neural_ws" \
      --entrypoint bash "$IMAGE" -c "set +u; source /opt/ros/humble/setup.bash; \
        source /integration/install_clean/setup.bash; cd /neural_ws; \
        colcon build --base-paths src --build-base build --install-base install \
          --packages-select larp_neural_depth larp_liteanystereo --symlink-install"
    ;;
  *) awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit 2 ;;
esac
