# Orin 移动操作主控平台

自建的主控软件栈：跑在 NVIDIA Jetson 上，控制底盘和机械臂，接雷达和相机，能用本地大模型把一句话变成动作。当前驱动的硬件是幻尔 ROSOrin Pro（Orin NX 16GB），以后换硬件只换适配器。

## 先读什么

1. [操作方案](操作方案.md)：每一步谁做、怎么做、怎么算过关。参与项目先读这份。
2. [技术路线](技术路线.md)：为什么这样设计。
3. [接口台账](adapters/rosorin/接口台账.md)：厂商出厂系统的接口。

## 目录

```
core/                    平台核心：纯 Python，不依赖 ROS 和具体硬件
  mmp_core/intents.py      意图校验器
adapters/rosorin/        练手车的适配，只留在实验室
  rrc_protocol/codec.py    控制板串口协议的编解码
  接口台账.md
```

包名前缀 `mmp` 是 mobile manipulation platform 的缩写。

厂家资料文件夹 `ROSOrin Pro智能小车(Orin NX)/` 也在本目录下，但已被 `.gitignore` 排除，不进仓库。

## 跑测试

在仓库根目录执行，需要 Python 3.10 以上，不需要安装任何第三方包：

```bash
python3 -m unittest discover -s core -t core
python3 -m unittest discover -s adapters/rosorin -t adapters/rosorin
```

## 三条规矩

1. 厂商代码不进仓库，密钥不进仓库。
2. 每次提交带测试；实测结果留记录。
3. 会让车或臂动的程序由人启动，车旁要有人。

完整的规矩见操作方案第 2 节。
