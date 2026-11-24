# 开发资料与发布范围

本仓库发布的是可检索的工程源码与配套文档，不是本机开发盘的镜像。

| 原目录 | GitHub 中的内容 |
| --- | --- |
| `Go_Car_V1/` | 上、下位两套工程及其中的 SDK、文档和许可。未发布编译输出、编辑器缓存、本机端口设置与绝对路径配置。 |
| `My_Study/` | `CS_yuanma` 学习快照及 `serial_diag`；前者保留与主工程不同的示例选择。`uart_monitor` 原本只有空目录，Git 不记录空目录。 |
| `huanjing/` | 本地工具链和语音板升级工具留在原目录；需要时按下表准备。 |
| `资料/` | 厂商离线资料包留在原目录，包括独立安装器、芯片手册合集、重复源码、CS100A 参考工程与讲解视频。 |

原始项目目录没有因发布而删除或改写。工程源码内的部分网络示例使用了疑似设备凭据，发布版将相关字段替换为占位符，并排除了 SDK 的 `ohos/libs/` 构建缓存；如需运行云端示例，请在本机使用自己的测试设备配置，不要把真实凭据提交到 Git。

## 工具和文档

| 需求 | 入口 |
| --- | --- |
| Hi3861 环境、上游代码 | [项目内上位系统说明](../Go_Car_V1/Up_system/README.md) · [Hi3861V100 工具链](https://hispark.obs.cn-east-3.myhuaweicloud.com/DevTools_Hi3861V100_v1.0.zip) · [上游 SDK](https://gitee.com/hispark/hi3861_hdu_iot_application) |
| FS-Hi3861 板级开发 | [板卡开发指导](<../Go_Car_V1/Up_system/src/vendor/hqyj/fs_hi3861/doc/华清远见 FS_Hi3861开发指导.md>) |
| STM32 电机板 | [Keil 工程](../Go_Car_V1/Down_system/MDK-ARM/FS-Hi3861-Motor-Driver.uvprojx) · [CubeMX 配置](../Go_Car_V1/Down_system/FS-Hi3861-Motor-Driver.ioc) |
| CS100A 传感器、语音板 | 向板卡厂商获取对应硬件版本配套的手册、参考固件与升级工具。 |

`Go_Car_V1/_apply.py` 是保留的历史脚本，带有旧工作目录和依赖旧文件行号的修改逻辑。它不是安装或构建步骤，不要在当前固件上直接运行。
