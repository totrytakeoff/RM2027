# 第三方来源与许可说明

文档更新日期：2026-07-19

本仓库包含按各自许可证分发的第三方代码。第三方源文件中保留的版权和
许可声明具有最终效力。

## 跃鹿战队 basic_framework

部分早期 BSP 与机器人模块来源于跃鹿战队开源 `basic_framework`。所参考的
上游发行版包含下列 MIT 许可文本；为避免法律文本语义偏差，此处保留英文原文。

历史 PID、CAN 通信、DJI 电机、电机定义和遥控源文件头中还曾包含
`Copyright (c) 2022 HNU YueLu EC all rights reserved`。相关战队标识统一在本文档
说明，使普通项目 API 和叙事保持中性，同时不丢失上游归属。
早期文件元数据中出现的 Wang Hongxi、Liu Wei、NeoZng 等作者信息也在此作为
历史来源记录；普通源码头和组件使用文档不再承担项目归属叙事。

```text
MIT License

Copyright (c) 2022 NeoZng

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

STM32 HAL、CMSIS、FreeRTOS、STM32 USB Device、CMSIS-DSP 和 SEGGER 源码位于
`third_party/`，并保留各自具有最终效力的许可声明。
