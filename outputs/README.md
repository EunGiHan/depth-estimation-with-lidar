```
rosbag에서 저장한 라이다 raw : outputs/<bag_num>/raw_pc/<frame_num>.npy
rosbag에서 저장한 이미지 raw : outputs/<bag_num>/raw_img/<frame_num>.png

투영된 라이다 포인트 배열: outputs/<bag_num>/depth_gt/depth_gt-<frame_num>.npy
투영된 라이다 포인트 사진: outputs/<bag_num>/depth_gt/depth_gt-<frame_num>.png
투영된 라이다 포인트와 이미지 정합 : outputs/<bag_num>/depth_gt/projection-<frame_num>.png

추정한 깊이 정보 배열: outputs/<bag_num>/depth_map/depth_map-<frame_num>.npy
추정한 깊이 정보 사진: outputs/<bag_num>/depth_map/depth_map-<frame_num>.png

평가 결과: outputs/<bag_num>/eval_result.txt
```

## outputs_sample에 예시가 있습니다!