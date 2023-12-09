## 验证标定精度


```shell
$ roslaunch verify_calibration verify_calibration.launch
```

```shell
$ rosrun verify_calibration verify_calibration_server
```

```shell
$ rosservice call /verify_calibration_pickup "call: {}"
```

```yaml
image_width: 640
image_height: 480
camera_name: gemini_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [454.9797058105469, 0.0, 329.6405029296875, 0.0, 454.9797058105469, 241.9853057861328, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 1e-10]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [454.9797058105469, 0.0, 329.6405029296875, 0.0, 0.0, 454.9797058105469, 241.9853057861328, 0.0, 0.0, 0.0, 1.0, 0.0]

```
