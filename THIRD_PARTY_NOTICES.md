# Third-Party Notices

This repository is released under the MIT License unless otherwise noted.

## Ultralytics YOLO segmentation weights

The realtime 3D segmentation pipeline is designed to work with Ultralytics YOLO segmentation weights such as `yolo11n-seg.pt`.

To keep repository licensing clear:

- the default weight file is **not tracked** in this repository
- this project does **not** claim ownership of Ultralytics model weights
- if you place a weight file such as `perception/tabletopseg3d/yolo11n-seg.pt` into your local checkout, that file remains governed by the upstream license terms

See the detailed subdirectory notice here:

- [perception/tabletopseg3d/licenses/ULTRALYTICS_YOLO11_NOTICE.md](./perception/tabletopseg3d/licenses/ULTRALYTICS_YOLO11_NOTICE.md)

Official upstream references:

- https://github.com/ultralytics/ultralytics
- https://github.com/ultralytics/assets
- https://www.ultralytics.com/license

## Repository hygiene notes

- Generated samples, runtime logs, and local model weights are intentionally excluded from version control.
- If more third-party binaries, models, or datasets are added later, update this file together with any subdirectory-specific notices.
