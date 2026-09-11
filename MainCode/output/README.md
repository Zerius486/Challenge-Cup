# 构建产物

本目录保存最近一次构建得到的可烧录文件，不参与编译：

- `firmware/`：主工程 `ChallengeCup_Main` 的 ELF、HEX、BIN 和 MAP。
- `standalone_tests/<module>/`：四个独立模块测试工程的 ELF、HEX、BIN 和 MAP。

重新构建后，脚本会覆盖对应文件。CMake 中间文件统一放在 `MainCode/build/`，不要手动把 build 目录中的文件复制到这里。
