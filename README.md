# rviz_color_code
RViz用の色見本
## 動作環境
* Ubuntu22.04
* ROS2 Humble

## 動作方法
```bash
ros2 launch rviz_color_code view.launch.py
```

## パラメータ
* **sphere_radius**: 球の半径
* **space_length**: 球と球の間の距離
* **column_size**: 球の列数（行数は個数から計算される）
* **division_number**: RGBそれぞれの分割数（10にすると、10x10x10の1000通りの色の球が作成される）
