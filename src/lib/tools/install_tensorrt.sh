
SRC_PATH="/home/trainerai/trainerai-core"
YOLO_PATH="/home/trainerai/trainerai-core/src/AI/object_detection/yolov5"
OBJ_DETECT_PATH="/home/trainerai/trainerai-core/src/AI/object_detection"
# MODEL_PATH = '/home/trainerai/trainerai-core/yolov5/yolov5m.engine'

if [[ ! "$PWD" = $SRC_PATH ]]
then
  echo "Source directory must be ${SRC_PATH}"
  exit -1
fi
echo "Install Tensorflow and Create engine"
# sudo apt-get install python3-libnvinfer-dev
# pip3 install nvidia-tensorrt -U --index-url https://pypi.ngc.nvidia.com
# python3 export.py --weights yolov5m.pt --include engine --batch_size 2

if [[ ! -d "$OBJ_DETECT_PATH/yolov5m.engine" ]]
then
    python3 "$YOLO_PATH/export.py" --weights "$OBJ_DETECT_PATH/yolov5m.pt" --include engine --batch-size 2 --device 0 --half
fi