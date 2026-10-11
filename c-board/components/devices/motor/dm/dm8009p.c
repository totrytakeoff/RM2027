#include "dm8009p.h"

/*
 * dm8009p.c 保留为兼容编译单元：
 * - 框架的 CMake 使用 GLOB_RECURSE 收集 modules 下的 .c 文件，删除该文件会导致旧 build 目录仍引用它而编译失败。
 * - 具体实现已迁移到通用 `dmmotor.c`，`dm8009p.h` 仅提供默认值与薄封装（inline）。
 */

