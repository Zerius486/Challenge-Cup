# 构建产物

本目录保存最近一次构建产生的固件和调试产物，不参与编译：

- `firmware/`：主工程 `ChallengeCup_Main` 的 ELF、HEX、BIN 和 MAP。
- `standalone_tests/<module>/`：六个独立模块测试工程的 ELF、HEX、BIN 和 MAP。

其中 ELF 用于调试器加载符号和单步调试，实际烧录使用 HEX 或 BIN；MAP 仅用于查看链接内存布局。测试固件的接线、运行现象和通过标准见 [`Projects/ModuleTests/README.md`](../Projects/ModuleTests/README.md)。

重新构建后，脚本会覆盖对应文件。CMake 中间文件统一放在 `MainCode/build/`，不要手动把 build 目录中的文件复制到这里。
