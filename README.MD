# ROS + Yolov5
## Realtime Yolov5 TensorRT inference with ROS

### Visualization
![demo](assets/demo.gif)  

### Return Results
detect results are in JSON string format
```
data: "{\"results\":[{\"id\":0,\"rect\":[945,501,18,35],\"score\":0.5213240385055542},{\"\
  id\":9,\"rect\":[334,235,29,74],\"score\":0.580041229724884}],\"timestamp\":\"1628653864.545534\"\
  }"
```

### Compile Environment
check below docker image for compiling
[https://hub.docker.com/repository/docker/willcy1006/tensorrt7.1](https://hub.docker.com/repository/docker/willcy1006/tensorrt7.1)
