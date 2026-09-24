# 分割模型

将 YOLO 分割权重放在此目录，默认文件名：

```
models/best.pt
```

当前权重已保留在 `best.pt`。嵌入式 `CirclePoseEngine` 会按
`../config/pose_params.yaml` 中的 `segmentation.model` 加载它。
