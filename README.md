# 基于同济大学sp_vision框架和和深圳大学开源模型的镖架制导
## 环境配置
理论上sp_vision能跑这个就能跑
##  运行
将src/main.cpp中硬编码的config_path改为自己的路径,执行以下命令
```bash
mkdir build
cmake ..
cmake -j
./standard
```
## 识别：
海康相机获取图像，openvino识别，多线程TBD

## 串口通信
TBD

## tips:
经测试，MV-CS016-10UC相机+8mm镜头 ，即自瞄常用的配置，在17米处和25米处均能稳定准确识别引导灯。
