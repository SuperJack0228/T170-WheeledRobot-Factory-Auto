# Python 示例程序

## 运行示例程序
1. 使用脚本运行
```bash
./start_py.sh
#支持传参
./start_py.sh lift_example.py 
```

2. 设置正确的环境变量后执行
```bash
# 注意，检查路径是否正确，根据包路径设置到 lib 库所在 
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:../../lib/arm64/22.04/
export PYTHONPATH=../../lib/arm64/22.04/:$PYTHONPATH
python3 main.py
```


### Pylance 语法检查

Pylance 提供严格的类型检查和语法分析。如需启用标准检查模式，可在 `.vscode/settings.json` 中添加：

```json
{
    "python.analysis.extraPaths": [
        // 注意，检查路径是否正确，根据包路径设置到 lib 库所在
        "${workspaceFolder}/lib/arm64/22.04",
    ],
    "python.languageServer": "Pylance",
    "python.analysis.typeCheckingMode": "basic",
    "python.analysis.autoImportCompletions": true,
}
```
