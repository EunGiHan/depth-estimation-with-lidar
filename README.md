# depth-estimation-with-lidar

## 폴더 구조
```
├─ .github/workflows            (github actions 관련 파일)
│   └─ black_formatter.yaml    (Black Formatter 적용)
├─ dataset/ (KITTI 데이터셋, 캘리브레이션 txt, 라이다 raw data, rosbag 등)
├─ evaluation_using_lidar/ (라이다 데이터를 활용한 추정 결과 평가)
├─ monocular_depth_estimation/ (단안카메라 깊이 추정 모델 관련)
├─ outputs/ (깊이 추정 결과, lidar 데이터 변환 결과, error 평가 결과 등)
├─ tools/ (기타 도구 모음 (ex. rosbag에서 이미지 & 라이다 데이터 수집해 저장하기))
```