# 模拟 LUT 下发表格记录

本文档记录本次通过 COM7 下发的模拟 LUT 数据，供上位机对照协议与表格内容。

## 外层协议

所有 LUT 数据帧使用原工程串口帧格式：

```text
3C 3C | CHANNEL | FLAGS | LENGTH_L LENGTH_H | PAYLOAD | CRC16_L CRC16_H | 3E 3E
```

LUT 数据通道：

```text
CHANNEL = 0x07
FLAGS   = 0x00
```

控制通道：

```text
CHANNEL = 0x06
FLAGS   = 0x00
```

控制 payload：

```text
01 00 = 暂停主动上传
01 01 = 恢复主动上传
```

压力模式标志位控制：

```text
CHANNEL = 0xFF
FLAGS   = 0x00
PAYLOAD = 00
含义：关闭压力模式，用于重新标定前清状态
```

## LUT Payload 格式

```text
PAYLOAD = CMD | COUNT | FLOAT_DATA[COUNT]
```

CMD 定义：

| CMD | package_id | 内容 |
|---:|---:|---|
| `0x03` | 0 | meta |
| `0x13` | 1 | area_cx |
| `0x23` | 2 | area_cy |
| `0x33` | 3 | force_grid |
| `0x43` | 4 | sensor_table |

float 均为 little-endian float32。`sensor_table` 展开顺序为：

```c
index = area_index * force_num + force_index;
```

## 本次下发顺序

```text
0x0F close press
pause upload
0x11 meta, area_cx, area_cy, force_grid, sensor_table
0x21 meta, area_cx, area_cy, force_grid, sensor_table
0x31 meta, area_cx, area_cy, force_grid, sensor_table
0x41 meta, area_cx, area_cy, force_grid, sensor_table
0x51 meta, area_cx, area_cy, force_grid, sensor_table
0x61 meta, area_cx, area_cy, force_grid, sensor_table
resume upload
```

## 通用 force_grid

6 个部位都使用同一组 `force_grid`：

```text
CMD   = 0x33
COUNT = 12
frame length = 60 bytes

[0, 52, 252, 452, 652, 852, 1052, 1252, 1452, 1652, 1852, 2052]
```

## sensor_table 生成规则

本次是模拟表格，用下面规则生成每个 area/force 的 sensor 值：

```text
if force_index == 0:
    sensor = 0
else:
    sensor = force_grid[force_index] * 2.5 + area_index * 45 + rows * cols * 1.5
```

所有值按 float32 下发。

## 0x11 拇指

### meta

```text
CMD   = 0x03
COUNT = 6
frame length = 36 bytes

[17, 6, 4, 4, 12, 0]
```

字段含义：

```text
part_id = 0x11
sensor_rows = 6
sensor_cols = 4
area_num = 4
force_num = 12
has_shape_feature = 0
```

### area_cx

```text
CMD   = 0x13
COUNT = 4
frame length = 28 bytes

[0.25, 0.75, 0.25, 0.75]
```

### area_cy

```text
CMD   = 0x23
COUNT = 4
frame length = 28 bytes

[0.25, 0.25, 0.75, 0.75]
```

### sensor_table

```text
CMD   = 0x43
COUNT = 48
frame length = 204 bytes
```

```text
area 0: [0, 166, 666, 1166, 1666, 2166, 2666, 3166, 3666, 4166, 4666, 5166]
area 1: [0, 211, 711, 1211, 1711, 2211, 2711, 3211, 3711, 4211, 4711, 5211]
area 2: [0, 256, 756, 1256, 1756, 2256, 2756, 3256, 3756, 4256, 4756, 5256]
area 3: [0, 301, 801, 1301, 1801, 2301, 2801, 3301, 3801, 4301, 4801, 5301]
```

## 0x21 食指

### meta

```text
CMD   = 0x03
COUNT = 6
frame length = 36 bytes

[33, 5, 4, 4, 12, 0]
```

字段含义：

```text
part_id = 0x21
sensor_rows = 5
sensor_cols = 4
area_num = 4
force_num = 12
has_shape_feature = 0
```

### area_cx

```text
CMD   = 0x13
COUNT = 4
frame length = 28 bytes

[0.25, 0.75, 0.25, 0.75]
```

### area_cy

```text
CMD   = 0x23
COUNT = 4
frame length = 28 bytes

[0.25, 0.25, 0.75, 0.75]
```

### sensor_table

```text
CMD   = 0x43
COUNT = 48
frame length = 204 bytes
```

```text
area 0: [0, 160, 660, 1160, 1660, 2160, 2660, 3160, 3660, 4160, 4660, 5160]
area 1: [0, 205, 705, 1205, 1705, 2205, 2705, 3205, 3705, 4205, 4705, 5205]
area 2: [0, 250, 750, 1250, 1750, 2250, 2750, 3250, 3750, 4250, 4750, 5250]
area 3: [0, 295, 795, 1295, 1795, 2295, 2795, 3295, 3795, 4295, 4795, 5295]
```

## 0x31 中指

### meta

```text
CMD   = 0x03
COUNT = 6
frame length = 36 bytes

[49, 5, 4, 4, 12, 0]
```

字段含义：

```text
part_id = 0x31
sensor_rows = 5
sensor_cols = 4
area_num = 4
force_num = 12
has_shape_feature = 0
```

### area_cx

```text
CMD   = 0x13
COUNT = 4
frame length = 28 bytes

[0.25, 0.75, 0.25, 0.75]
```

### area_cy

```text
CMD   = 0x23
COUNT = 4
frame length = 28 bytes

[0.25, 0.25, 0.75, 0.75]
```

### sensor_table

```text
CMD   = 0x43
COUNT = 48
frame length = 204 bytes
```

```text
area 0: [0, 160, 660, 1160, 1660, 2160, 2660, 3160, 3660, 4160, 4660, 5160]
area 1: [0, 205, 705, 1205, 1705, 2205, 2705, 3205, 3705, 4205, 4705, 5205]
area 2: [0, 250, 750, 1250, 1750, 2250, 2750, 3250, 3750, 4250, 4750, 5250]
area 3: [0, 295, 795, 1295, 1795, 2295, 2795, 3295, 3795, 4295, 4795, 5295]
```

## 0x41 无名指

### meta

```text
CMD   = 0x03
COUNT = 6
frame length = 36 bytes

[65, 5, 4, 4, 12, 0]
```

字段含义：

```text
part_id = 0x41
sensor_rows = 5
sensor_cols = 4
area_num = 4
force_num = 12
has_shape_feature = 0
```

### area_cx

```text
CMD   = 0x13
COUNT = 4
frame length = 28 bytes

[0.25, 0.75, 0.25, 0.75]
```

### area_cy

```text
CMD   = 0x23
COUNT = 4
frame length = 28 bytes

[0.25, 0.25, 0.75, 0.75]
```

### sensor_table

```text
CMD   = 0x43
COUNT = 48
frame length = 204 bytes
```

```text
area 0: [0, 160, 660, 1160, 1660, 2160, 2660, 3160, 3660, 4160, 4660, 5160]
area 1: [0, 205, 705, 1205, 1705, 2205, 2705, 3205, 3705, 4205, 4705, 5205]
area 2: [0, 250, 750, 1250, 1750, 2250, 2750, 3250, 3750, 4250, 4750, 5250]
area 3: [0, 295, 795, 1295, 1795, 2295, 2795, 3295, 3795, 4295, 4795, 5295]
```

## 0x51 小指

### meta

```text
CMD   = 0x03
COUNT = 6
frame length = 36 bytes

[81, 5, 4, 4, 12, 0]
```

字段含义：

```text
part_id = 0x51
sensor_rows = 5
sensor_cols = 4
area_num = 4
force_num = 12
has_shape_feature = 0
```

### area_cx

```text
CMD   = 0x13
COUNT = 4
frame length = 28 bytes

[0.25, 0.75, 0.25, 0.75]
```

### area_cy

```text
CMD   = 0x23
COUNT = 4
frame length = 28 bytes

[0.25, 0.25, 0.75, 0.75]
```

### sensor_table

```text
CMD   = 0x43
COUNT = 48
frame length = 204 bytes
```

```text
area 0: [0, 160, 660, 1160, 1660, 2160, 2660, 3160, 3660, 4160, 4660, 5160]
area 1: [0, 205, 705, 1205, 1705, 2205, 2705, 3205, 3705, 4205, 4705, 5205]
area 2: [0, 250, 750, 1250, 1750, 2250, 2750, 3250, 3750, 4250, 4750, 5250]
area 3: [0, 295, 795, 1295, 1795, 2295, 2795, 3295, 3795, 4295, 4795, 5295]
```

## 0x61 手掌

### meta

```text
CMD   = 0x03
COUNT = 6
frame length = 36 bytes

[97, 16, 16, 5, 12, 0]
```

字段含义：

```text
part_id = 0x61
sensor_rows = 16
sensor_cols = 16
area_num = 5
force_num = 12
has_shape_feature = 0
```

### area_cx

```text
CMD   = 0x13
COUNT = 5
frame length = 32 bytes

[0.2, 0.8, 0.2, 0.8, 0.5]
```

### area_cy

```text
CMD   = 0x23
COUNT = 5
frame length = 32 bytes

[0.2, 0.2, 0.8, 0.8, 0.5]
```

### sensor_table

```text
CMD   = 0x43
COUNT = 60
frame length = 252 bytes
```

```text
area 0: [0, 514, 1014, 1514, 2014, 2514, 3014, 3514, 4014, 4514, 5014, 5514]
area 1: [0, 559, 1059, 1559, 2059, 2559, 3059, 3559, 4059, 4559, 5059, 5559]
area 2: [0, 604, 1104, 1604, 2104, 2604, 3104, 3604, 4104, 4604, 5104, 5604]
area 3: [0, 649, 1149, 1649, 2149, 2649, 3149, 3649, 4149, 4649, 5149, 5649]
area 4: [0, 694, 1194, 1694, 2194, 2694, 3194, 3694, 4194, 4694, 5194, 5694]
```

## 本次实际发送帧长度汇总

```text
0x0F close press         11 bytes
pause upload             12 bytes
0x11 meta                36 bytes
0x11 area_cx             28 bytes
0x11 area_cy             28 bytes
0x11 force_grid          60 bytes
0x11 sensor_table       204 bytes
0x21 meta                36 bytes
0x21 area_cx             28 bytes
0x21 area_cy             28 bytes
0x21 force_grid          60 bytes
0x21 sensor_table       204 bytes
0x31 meta                36 bytes
0x31 area_cx             28 bytes
0x31 area_cy             28 bytes
0x31 force_grid          60 bytes
0x31 sensor_table       204 bytes
0x41 meta                36 bytes
0x41 area_cx             28 bytes
0x41 area_cy             28 bytes
0x41 force_grid          60 bytes
0x41 sensor_table       204 bytes
0x51 meta                36 bytes
0x51 area_cx             28 bytes
0x51 area_cy             28 bytes
0x51 force_grid          60 bytes
0x51 sensor_table       204 bytes
0x61 meta                36 bytes
0x61 area_cx             32 bytes
0x61 area_cy             32 bytes
0x61 force_grid          60 bytes
0x61 sensor_table       252 bytes
resume upload            12 bytes
```
