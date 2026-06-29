#!/usr/bin/env bash
set -euo pipefail

# ---------------------------------------------------------------------------
# Export YOLOv8 -> ONNX -> TensorRT (.engine) pour NVIDIA Jetson
# ---------------------------------------------------------------------------
# Usage :
#   ./scripts/convert_yolo_tensorrt.sh chemin/vers/best.pt
#   ./scripts/convert_yolo_tensorrt.sh              # default : best.pt
# ---------------------------------------------------------------------------

MODEL_PATH="${1:-best.pt}"
BASENAME=$(basename "$MODEL_PATH" .pt)
DIR=$(dirname "$MODEL_PATH")

echo "=== [1/3] Export YOLO -> ONNX ==="
yolo export model="$MODEL_PATH" format=onnx imgsz=640 opset=12

ONNX_FILE="$DIR/$BASENAME.onnx"
if [ ! -f "$ONNX_FILE" ]; then
    echo "ERREUR : $ONNX_FILE introuvable apres export ONNX."
    exit 1
fi

echo "=== [2/3] ONNX -> TensorRT (FP16) ==="
# Sur Jetson trtexec est souvent ici ; sinon on espere qu'il est dans le PATH
TRTEXEC=$(command -v trtexec || echo "/usr/src/tensorrt/bin/trtexec")

$TRTEXEC \
    --onnx="$ONNX_FILE" \
    --saveEngine="$DIR/$BASENAME.engine" \
    --fp16 \
    --workspace=1024

echo "=== [3/3] Terminé ==="
echo "Moteur TensorRT : $DIR/$BASENAME.engine"
echo ""
echo "Pour utiliser directement avec Ultralytics :"
echo "  from ultralytics import YOLO"
echo "  model = YOLO('$DIR/$BASENAME.engine')"
