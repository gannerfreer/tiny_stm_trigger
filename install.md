# 编译说明（CMake）

## 1) 依赖安装

```bash
sudo apt update
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi cmake make
sudo apt install -y ninja-build
```

验证：

```bash
arm-none-eabi-gcc --version
cmake --version
```

---

## 2) 用 Unix Makefiles 编译

```bash
cmake -S . -B build-make -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build-make -j
```
编译成功后，会提示：

``` bash
[100%] Linking C executable Stm32.elf
Memory region         Used Size  Region Size  %age Used
             RAM:        1864 B         4 KB     45.51%
           FLASH:       10412 B        32 KB     31.77%
[100%] Built target Stm32
```

---

## 3) 用 Ninja 编译

```bash
cmake -S . -B build-ninja -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build-ninja -j
```

编译成功后，会提示：

``` bash
[25/25] Linking C executable Stm32.elf
Memory region         Used Size  Region Size  %age Used
             RAM:        1864 B         4 KB     45.51%
           FLASH:       10416 B        32 KB     31.79%
```

---

## 4) Make 和 Ninja 的区别

- Ninja 一般构建更迅速；Make 偏默认，两者只用其一即可。

