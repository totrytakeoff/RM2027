# univsersal communication

@todo

unicomm旨在为通信提供一套标准的协议接口，屏蔽底层的硬件差异，使得上层应用可以定制通信协议，包括包长度/可变帧长/帧头尾/校验方式等。

不论底层具体使用的是什么硬件接口，实际上每一帧传输完并把数据放在缓冲区之后，就没有任何区别了。 此模块实际上就是对缓冲区的rawdata进行操作，包括查找帧头，计算包长度，校验错误等。

正式视觉帧协议已经由 `vision_link` / `vision_protocol` 进入
`rm_components_runtime`；旧的 `master_process` / `serial_frame_protocol` 只保留在
`rm_components_compat` 供历史 demo 使用。后续统一传输服务的工作应先评估兼容路径的
调用者，再决定迁移或删除，不能把旧协议重新带回正式 runtime。
