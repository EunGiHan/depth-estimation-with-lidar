# directory tree and file name format

```
outputs/
├─ <bag_num>/
│   ├─ raw_pc/
│   │   └─ <frame_num>.npy      (rosbag에서 저장한 라이다 raw)
│   ├─ raw_img/
│   │   └─ <frame_num>.png      (rosbag에서 저장한 이미지 raw)
│   ├─ undist_img/
│   │   └─ <frame_num>.png      (왜곡 제거 이미지)
│   │
│   ├─ depth_gt/
│   │   ├─ depth_gt-<frame_num>.npy     (투영된 라이다 포인트 배열)
│   │   ├─ depth_gt-<frame_num>.png     (투영된 라이다 포인트 사진)
│   │   └─ projection-<frame_num>.png   (투영된 라이다 포인트와 이미지 정합)
│   │
│   ├─ depth_map/
│   │   ├─ depth_map-<frame_num>.npy    (추정한 깊이 정보 배열)
│   │   └─ depth_map-<frame_num>.png    (추정한 깊이 정보 사진)
│   │
|   └── eval_result.txt          (평가 결과)
|
├─ <bag_num>/
|   ...
```
