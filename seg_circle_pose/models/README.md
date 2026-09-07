# 分割模型

将 YOLO 分割权重放在此目录，默认文件名：

```
models/best.pt
```

首次部署可从训练产物复制，例如：

```bash
cp /path/to/train/weights/best.pt models/best.pt
```

`main.py` 与 `AlgorithmConfig` 未指定 `model_path` 时会自动加载该文件。
