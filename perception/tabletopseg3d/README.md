# TabletopSeg3D

Chinese documentation: [README_cn.md](./README_cn.md)

Realtime desktop-object 3D detection based on `Intel RealSense + YOLO Segmentation + Open3D`.

![TabletopSeg3D demo](photo/1.png)

This repository is intentionally trimmed down to the final runtime features:

- realtime Open3D scene visualization
- headless JSON output

## Features

- single-camera realtime pipeline, tested on `D435I` and `D405`
- YOLO instance segmentation
- depth-to-point-cloud projection
- tabletop-aligned OBB
  - box `Z` follows the tabletop normal
  - only one rotational degree of freedom is reported: `yaw`
- optional 3D labels in Open3D
- `--no-display` prints per-frame JSON including:
  - object class
  - center in camera coordinates
  - box size
  - box yaw

## Layout

```text
perception/tabletopseg3d/
├── README.md
├── README_cn.md
├── licenses/
│   └── ULTRALYTICS_YOLO11_NOTICE.md
├── requirements.txt
├── scripts/
│   └── realtime_open3d_scene.py
└── src/
    ├── camera/
    │   └── realsense_capture.py
    ├── geometry/
    │   └── pointcloud.py
    ├── segmentation/
    │   └── runtime.py
    └── visualization/
        └── open3d_scene.py
```

## Environment

Python `3.11` is recommended.

## Quick Start

### Step 1: Prepare a Python environment

Using an isolated environment is recommended, and Python `3.11` is the preferred version.

For example:

```bash
conda create -n tabletopseg3d python=3.11
conda activate tabletopseg3d
```

or:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
```

### Step 2: Install system-side dependencies

If your system does not already provide the required `librealsense` runtime, install Intel RealSense SDK first.

This step is not required for documentation-only work, but it is required before running the demo with a depth camera.

### Step 3: Install Python dependencies

The default repository setup is a **CPU-oriented** installation path and is the best starting point if you want to get the pipeline working first.

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python -m pip install -r requirements.txt
```

### CPU users

The default command above is enough.

Notes:

- the current `requirements.txt` installs CPU builds of `torch` and `torchvision`
- this is the recommended path for first-time setup, documentation work, and general validation

### GPU users

If you want GPU acceleration for YOLO inference, do not stop at the default dependency set. Use a separate GPU-oriented install branch instead.

First make sure your machine already has:

- an NVIDIA driver
- a compatible CUDA runtime

Recommended installation order:

1. Install the common dependencies first, excluding `torch` and `torchvision`

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python -m pip install numpy==2.4.3 opencv-python==4.13.0.92 open3d==0.19.0 \
  pyserial pyrealsense2==2.56.5.9235 fashionstar-uart-sdk ultralytics==8.4.24
```

2. Install GPU builds of `torch` and `torchvision` that match your CUDA stack

Example pattern:

```bash
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cuXXX
```

Replace `cuXXX` with the CUDA build that matches your local environment, for example `cu121` or `cu124`.

### GPU notes

- the key requirement for GPU inference is replacing CPU `torch` / `torchvision` with CUDA-matched builds
- `ultralytics` itself does not guarantee GPU execution; CUDA support comes from the installed PyTorch build
- if the CUDA version and the PyTorch wheel do not match, inference may silently fall back to CPU or fail at import/runtime
- if you are unsure which CUDA build to choose, verify your local driver and CUDA environment before installing PyTorch

### Recommended path

- use the CPU branch if you want the fastest and safest first setup
- use the GPU branch only after confirming your CUDA environment is ready
- for project documentation, it is best to keep CPU and GPU installation paths as two clearly separated branches

## List Connected Cameras

To print the model name and serial number of connected RealSense devices:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py --list-devices
```

Example output:

```text
Connected RealSense devices:
- Intel RealSense D405 | serial=409122273421
- Intel RealSense D435I | serial=419522072950
```

## Model

The default runtime examples use the model name:

```bash
yolo11n-seg.pt
```

You can replace it with your own YOLO segmentation model through `--model`.

Important:

- this repository does **not** track `yolo11n-seg.pt` in git
- if you want to use the default command form, place `yolo11n-seg.pt` in the repository root yourself
- the weight file is a third-party Ultralytics asset; see [licenses/ULTRALYTICS_YOLO11_NOTICE.md](./licenses/ULTRALYTICS_YOLO11_NOTICE.md) and [../../THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md) before redistributing it

Accepted model forms:

- an official model name such as `yolo11n-seg.pt`
- a local weight file such as `./my_model.pt`
- a training output such as `runs/segment/train/weights/best.pt`
- any absolute path such as `/home/yourname/models/best.pt`

Recommended usage with your own model:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 419522072950 \
  --model runs/segment/train/weights/best.pt \
  --device cpu
```

Headless output with your own model:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 419522072950 \
  --model runs/segment/train/weights/best.pt \
  --device cpu \
  --frames 10 \
  --no-display
```

Important:

- you must use a `segmentation` model
- a plain detection model has no masks and cannot produce the 3D box pipeline used here

## Realtime Visualization

Default example with `D435I`:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 419522072950 \
  --device cpu
```

With 3D labels:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 419522072950 \
  --device cpu \
  --show-labels
```

## Headless Output

Run without GUI and print one JSON record per frame:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 419522072950 \
  --device cpu \
  --frames 10 \
  --no-display
```

Example JSON:

```json
{
  "frame_index": 0,
  "fps": 16.36,
  "infer_ms": 46.28,
  "geom_ms": 7.73,
  "scene_point_count": 67930,
  "table_normal_xyz": [0.039279, -0.227524, -0.97298],
  "detections": [
    {
      "class_name": "banana",
      "confidence": 0.792091,
      "center_camera_xyz_m": [0.031458, 0.085936, 0.402289],
      "extent_xyz_m": [0.17001, 0.064569, 0.040624],
      "yaw_rad": -0.826237,
      "yaw_deg": -47.3399,
      "point_count": 14090
    }
  ]
}
```

## D405 Example

`D405` is more suitable for close range. A tighter depth range is recommended:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 409122273421 \
  --device cpu \
  --min-depth 0.02 \
  --max-depth 0.50
```

Headless:

```bash
cd /home/misca/TabletopGraspSystem/perception/tabletopseg3d
python scripts/realtime_open3d_scene.py \
  --serial 409122273421 \
  --device cpu \
  --min-depth 0.02 \
  --max-depth 0.50 \
  --frames 10 \
  --no-display
```

## Main Arguments

- `--list-devices`: print connected RealSense devices and exit
- `--serial`: select the RealSense serial number
- `--model`: select the YOLO segmentation model
- `--device`: inference device, currently `cpu` is recommended
- `--imgsz`: YOLO input size
- `--min-depth` / `--max-depth`: valid depth range
- `--target-class`: keep only one target class
- `--show-labels`: enable 3D labels
- `--show-object-points`: highlight object points in the scene point cloud
- `--no-display`: disable visualization and print per-frame JSON
- `--frames`: stop after a fixed number of frames

## Notes

- the tabletop normal is estimated once at startup
- the 3D box is a tabletop-aligned OBB
- only one rotational degree of freedom is reported: `yaw`
- `yaw` is defined in the tabletop plane basis and is suitable for grasp filtering and pose screening
