## 杂记

代码考虑到d435i的带宽占用，导致原始的 infra 红外流因为无法传输而“饿死”(no message)的情况，将深度图像对齐移到主板计算
可考虑修改< arg name="emitter_enable" default="true"/ >
