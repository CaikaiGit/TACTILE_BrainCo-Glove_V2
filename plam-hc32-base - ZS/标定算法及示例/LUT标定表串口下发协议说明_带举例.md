# LUT 标定表串口下发协议说明（带举例版）

## 1. 文档目的

本文档用于说明上位机如何通过现有串口通信协议向下位机/MCU 下发力觉 LUT 标定表。

当前协议不重新设计新的大协议，而是在已有 `calibration_curve(float *buf, size_t count, int package_id)` 基础上扩展：

```text
package_id = 0：部位元信息 meta
package_id = 1：area_cx
package_id = 2：area_cy
package_id = 3：force_grid
package_id = 4：sensor_table
```

一个部位完整下发顺序为：

```text
meta → area_cx → area_cy → force_grid → sensor_table
```

多个部位则按部位循环下发：

```text
暂停主动上传

0x11: meta → area_cx → area_cy → force_grid → sensor_table
0x21: meta → area_cx → area_cy → force_grid → sensor_table
0x31: meta → area_cx → area_cy → force_grid → sensor_table
0x41: meta → area_cx → area_cy → force_grid → sensor_table
0x51: meta → area_cx → area_cy → force_grid → sensor_table
0x61: meta → area_cx → area_cy → force_grid → sensor_table

恢复主动上传
```

如果某个部位没有对应标定 CSV，上位机会跳过该部位。

---

## 2. 外层通信帧格式

每一帧的外层格式如下：

```text
3C 3C | CHANNEL | LENGTH | PAYLOAD | CHECKSUM | 3E 3E
```

| 字段 | 长度 | 字节序 | 说明 |
|---|---:|---|---|
| Frame Header | 2 bytes | 固定 | 帧头，固定为 `3C 3C` |
| Channel | 2 bytes | Little-endian | 通道号 |
| Length | 2 bytes | Little-endian | Payload 长度，单位 byte |
| Payload | N bytes | - | 有效负载 |
| Checksum | 2 bytes | Little-endian | 校验值 |
| Frame Tail | 2 bytes | 固定 | 帧尾，固定为 `3E 3E` |

### 2.1 外层帧举例

原始数据：

```text
3C 3C 07 00 1A 00 03 06 00 00 88 41 00 00 80 40 00 00 C0 40 00 00 40 40 00 00 40 41 00 00 80 3F AC 38 3E 3E
```

拆解：

```text
3C 3C      帧头
07 00      Channel = 0x0007
1A 00      Payload 长度 = 0x001A = 26 bytes
03 06 ...  Payload
AC 38      Checksum
3E 3E      帧尾
```

其中 `Channel = 0x0007` 表示这是 LUT 标定数据下发帧。

---

## 3. Channel 定义

当前示例中出现两个通道。

| Channel | 名称 | 用途 |
|---:|---|---|
| `0x0006` | 控制通道 | 暂停/恢复主动上传 |
| `0x0007` | LUT 标定通道 | 下发 meta、area_cx、area_cy、force_grid、sensor_table |

---

## 4. 控制帧说明

控制帧使用：

```text
Channel = 0x0006
```

Payload 固定为 2 bytes：

```text
01 XX
```

| Payload | 含义 |
|---|---|
| `01 00` | 暂停主动上传 |
| `01 01` | 恢复主动上传 |

---

### 4.1 暂停主动上传示例

原始帧：

```text
3C 3C 06 00 02 00 01 00 01 90 3E 3E
```

拆解：

```text
3C 3C      帧头
06 00      Channel = 0x0006
02 00      Payload 长度 = 2
01 00      Payload
01 90      Checksum
3E 3E      帧尾
```

含义：

```text
暂停主动上传
```

---

### 4.2 恢复主动上传示例

原始帧：

```text
3C 3C 06 00 02 00 01 01 C0 50 3E 3E
```

拆解：

```text
3C 3C      帧头
06 00      Channel = 0x0006
02 00      Payload 长度 = 2
01 01      Payload
C0 50      Checksum
3E 3E      帧尾
```

含义：

```text
恢复主动上传
```

---

## 5. LUT 数据帧说明

LUT 数据帧使用：

```text
Channel = 0x0007
```

Payload 格式如下：

```text
CMD | COUNT | FLOAT_DATA[COUNT]
```

| 字段 | 长度 | 说明 |
|---|---:|---|
| CMD | 1 byte | 命令字，包含 package_id |
| COUNT | 1 byte | 后续 float32 数量 |
| FLOAT_DATA | COUNT × 4 bytes | 小端 float32 数组 |

Payload 长度计算：

```text
Length = 2 + COUNT × 4
```

CMD 计算方式：

```text
CMD = (package_id << 4) | 0x03
```

所以：

| package_id | CMD | 含义 |
|---:|---:|---|
| 0 | `0x03` | meta，部位元信息 |
| 1 | `0x13` | area_cx |
| 2 | `0x23` | area_cy |
| 3 | `0x33` | force_grid |
| 4 | `0x43` | sensor_table |

---

## 6. float32 小端解析举例

协议里的 LUT 数据使用 little-endian float32。

例如：

```text
00 00 88 41
```

按 little-endian float32 解析为：

```text
17.0
```

因为 `17` 的 IEEE754 float32 小端表示就是：

```text
00 00 88 41
```

再比如：

```text
00 00 50 42
```

解析为：

```text
52.0
```

因此下位机解析 float 时需要注意字节序。

---

## 7. package_id = 0：meta 包

meta 包用于说明后续 1~4 号包属于哪个部位，以及该部位 LUT 的尺寸。

meta 固定 6 个 float：

```text
meta[0] = part_id
meta[1] = sensor_rows
meta[2] = sensor_cols
meta[3] = area_num
meta[4] = force_num
meta[5] = has_shape_feature
```

| 下标 | 字段 | 说明 |
|---:|---|---|
| 0 | part_id | 部位协议 ID，例如 `0x11`、`0x21`、`0x61` |
| 1 | sensor_rows | 传感器矩阵行数 |
| 2 | sensor_cols | 传感器矩阵列数 |
| 3 | area_num | 标定区域数量 |
| 4 | force_num | 力值梯度数量 |
| 5 | has_shape_feature | 是否启用扩展特征表；基础 LUT 建议为 `0` |

> 注意：虽然 meta 使用 float 发送，但这些字段都应该按整数解释。

---

## 8. package_id = 1：area_cx

```text
area_cx[area_num]
```

说明：

- 每个标定区域的归一化 x 坐标；
- 类型为 float32；
- 数组长度必须等于 `area_num`。

示例：

```text
area_cx = [0.9272, 0.5173, 0.1003]
```

表示当前部位有 3 个标定区域，每个区域都有一个中心 x 坐标。

---

## 9. package_id = 2：area_cy

```text
area_cy[area_num]
```

说明：

- 每个标定区域的归一化 y 坐标；
- 类型为 float32；
- 数组长度必须等于 `area_num`。

示例：

```text
area_cy = [0.7538, 0.5314, 0.7690]
```

---

## 10. package_id = 3：force_grid

```text
force_grid[force_num]
```

说明：

- 力值梯度；
- 单位通常为 g；
- 类型为 float32；
- 数组长度必须等于 `force_num`。

示例：

```text
force_grid = [
  0,
  52,
  252,
  452,
  652,
  852,
  1052,
  1252,
  1452,
  1652,
  1852,
  2052
]
```

该示例表示该部位有 12 个压力梯度。

---

## 11. package_id = 4：sensor_table

```text
sensor_table[area_num × force_num]
```

说明：

- 每个标定区域在每个压力梯度下对应的传感器响应值；
- 当前一般为 `sensor_sum`；
- 类型为 float32；
- 长度必须等于 `area_num × force_num`。

展开顺序必须为：

```text
sensor_table[area][force]
```

即：

```c
index = area_index * force_num + force_index;
```

例如：

```text
area_num = 3
force_num = 12
```

则 `sensor_table` 长度应为：

```text
3 × 12 = 36
```

展开方式：

```text
area 0:
  force0, force1, force2, ..., force11

area 1:
  force0, force1, force2, ..., force11

area 2:
  force0, force1, force2, ..., force11
```

---

# 12. 完整示例一：部位 0x11

以下示例来自实际抓包数据。

---

## 12.1 0x11 meta 帧

原始帧：

```text
3C 3C 07 00 1A 00 03 06 00 00 88 41 00 00 80 40 00 00 C0 40 00 00 40 40 00 00 40 41 00 00 80 3F AC 38 3E 3E
```

外层解析：

```text
Channel = 0x0007
Length  = 0x001A = 26 bytes
Payload = 03 06 + 6 个 float
```

Payload 解析：

```text
CMD   = 0x03
COUNT = 0x06 = 6
```

由 `CMD = 0x03` 得到：

```text
package_id = 0
含义 = meta
```

float 数据解析：

| 字节 | float值 | 含义 |
|---|---:|---|
| `00 00 88 41` | 17 | part_id = `0x11` |
| `00 00 80 40` | 4 | sensor_rows = 4 |
| `00 00 C0 40` | 6 | sensor_cols = 6 |
| `00 00 40 40` | 3 | area_num = 3 |
| `00 00 40 41` | 12 | force_num = 12 |
| `00 00 80 3F` | 1 | has_shape_feature = 1 |

结论：

```text
当前开始下发部位 0x11 的 LUT。
该部位矩阵尺寸为 4 × 6。
该部位有 3 个标定区域。
该部位有 12 个压力梯度。
```

---

## 12.2 0x11 area_cx 帧

原始帧：

```text
3C 3C 07 00 0E 00 13 03 40 60 6D 3F F0 70 04 3F FB 5D CD 3D 31 2E 3E 3E
```

解析：

```text
Channel = 0x0007
Length  = 0x000E = 14 bytes
CMD     = 0x13
COUNT   = 3
```

由 `CMD = 0x13` 得到：

```text
package_id = 1
含义 = area_cx
```

数据：

```text
area_cx = [
  0.9272,
  0.5173,
  0.1003
]
```

检查：

```text
COUNT = 3
area_num = 3
结果：通过
```

---

## 12.3 0x11 area_cy 帧

原始帧：

```text
3C 3C 07 00 0E 00 23 03 42 FB 40 3F DC 0A 08 3F 89 DC 44 3F C4 28 3E 3E
```

解析：

```text
CMD   = 0x23
COUNT = 3
package_id = 2
含义 = area_cy
```

数据：

```text
area_cy = [
  0.7538,
  0.5314,
  0.7690
]
```

检查：

```text
COUNT = 3
area_num = 3
结果：通过
```

---

## 12.4 0x11 force_grid 帧

原始帧：

```text
3C 3C 07 00 32 00 33 0C 00 00 00 00 00 00 50 42 00 00 7C 43 00 00 E2 43 00 00 23 44 00 00 55 44 00 80 83 44 00 80 9C 44 00 80 B5 44 00 80 CE 44 00 80 E7 44 00 40 00 45 CB 76 3E 3E
```

解析：

```text
CMD   = 0x33
COUNT = 12
package_id = 3
含义 = force_grid
```

数据：

```text
force_grid = [
  0,
  52,
  252,
  452,
  652,
  852,
  1052,
  1252,
  1452,
  1652,
  1852,
  2052
]
```

检查：

```text
COUNT = 12
force_num = 12
结果：通过
```

---

## 12.5 0x11 sensor_table 帧

原始帧开头：

```text
3C 3C 07 00 92 00 43 24 ...
```

解析：

```text
Channel = 0x0007
Length  = 0x0092 = 146 bytes
CMD     = 0x43
COUNT   = 0x24 = 36
```

由 `CMD = 0x43` 得到：

```text
package_id = 4
含义 = sensor_table
```

理论长度：

```text
area_num × force_num = 3 × 12 = 36
```

检查：

```text
COUNT = 36
理论值 = 36
结果：通过
```

展开为：

```text
area 0:
[0, 41, 194, 286, 347, 409, 452, 495, 540, 569, 627, 712]

area 1:
[0, 71, 247, 358, 421, 452, 495, 536, 539, 546, 572, 592]

area 2:
[0, 79, 190, 312, 423, 469, 532, 584, 629, 663, 711, 739]
```

接收完 `package_id = 4` 后，可以认为部位 `0x11` 的基础 LUT 数据已接收完整。

---

# 13. 完整示例二：部位 0x61

---

## 13.1 0x61 meta 帧

原始帧：

```text
3C 3C 07 00 1A 00 03 06 00 00 C2 42 00 00 80 41 00 00 80 41 00 00 A0 40 00 00 40 41 00 00 00 00 C6 57 3E 3E
```

解析：

```text
CMD   = 0x03
COUNT = 6
package_id = 0
含义 = meta
```

float 数据：

| 字节 | float值 | 含义 |
|---|---:|---|
| `00 00 C2 42` | 97 | part_id = `0x61` |
| `00 00 80 41` | 16 | sensor_rows = 16 |
| `00 00 80 41` | 16 | sensor_cols = 16 |
| `00 00 A0 40` | 5 | area_num = 5 |
| `00 00 40 41` | 12 | force_num = 12 |
| `00 00 00 00` | 0 | has_shape_feature = 0 |

结论：

```text
当前开始下发部位 0x61 的 LUT。
该部位矩阵尺寸为 16 × 16。
该部位有 5 个标定区域。
该部位有 12 个压力梯度。
```

---

## 13.2 0x61 area_cx 帧

原始帧：

```text
3C 3C 07 00 16 00 13 05 B1 83 BD 3E 21 69 29 3E 31 BF 00 3F 36 35 96 3E B5 E1 D0 3D 01 B8 3E 3E
```

解析：

```text
CMD   = 0x13
COUNT = 5
package_id = 1
含义 = area_cx
```

数据：

```text
area_cx = [
  0.3701,
  0.1654,
  0.5029,
  0.2934,
  0.1020
]
```

检查：

```text
COUNT = 5
area_num = 5
结果：通过
```

---

## 13.3 0x61 area_cy 帧

原始帧：

```text
3C 3C 07 00 16 00 23 05 7C D0 5E 3F 3D 93 59 3F 1F 59 EF 3E 54 5F F7 3E 4B FE FD 3E 0C 5F 3E 3E
```

解析：

```text
CMD   = 0x23
COUNT = 5
package_id = 2
含义 = area_cy
```

数据：

```text
area_cy = [
  0.8704,
  0.8499,
  0.4675,
  0.4831,
  0.4961
]
```

检查：

```text
COUNT = 5
area_num = 5
结果：通过
```

---

## 13.4 0x61 force_grid 帧

原始帧：

```text
3C 3C 07 00 32 00 33 0C 00 00 00 00 00 00 50 42 00 00 7C 43 00 00 E2 43 00 00 23 44 00 00 55 44 00 80 83 44 00 80 9C 44 00 80 B5 44 00 80 CE 44 00 80 E7 44 00 40 00 45 CB 76 3E 3E
```

解析：

```text
CMD   = 0x33
COUNT = 12
package_id = 3
含义 = force_grid
```

数据：

```text
force_grid = [
  0,
  52,
  252,
  452,
  652,
  852,
  1052,
  1252,
  1452,
  1652,
  1852,
  2052
]
```

检查：

```text
COUNT = 12
force_num = 12
结果：通过
```

---

## 13.5 0x61 sensor_table 帧

原始帧开头：

```text
3C 3C 07 00 F2 00 43 3C ...
```

解析：

```text
Channel = 0x0007
Length  = 0x00F2 = 242 bytes
CMD     = 0x43
COUNT   = 0x3C = 60
```

由 `CMD = 0x43` 得到：

```text
package_id = 4
含义 = sensor_table
```

理论长度：

```text
area_num × force_num = 5 × 12 = 60
```

检查：

```text
COUNT = 60
理论值 = 60
结果：通过
```

展开为：

```text
area 0:
[0, 172, 380, 484, 556, 622, 653, 710, 762, 797, 807, 823]

area 1:
[0, 144, 315, 421, 493, 583, 644, 717, 748, 777, 786, 810]

area 2:
[0, 125, 333, 463, 526, 611, 636, 705, 737, 793, 816, 817]

area 3:
[0, 139, 319, 423, 519, 541, 620, 635, 669, 716, 738, 752]

area 4:
[0, 133, 316, 426, 507, 548, 572, 607, 634, 642, 656, 667]
```

接收完 `package_id = 4` 后，可以认为部位 `0x61` 的基础 LUT 数据已接收完整。

---

# 14. 下位机推荐解析流程

下位机接收流程建议如下：

```text
收到一帧
  ↓
检查帧头 3C 3C
  ↓
解析 Channel
  ↓
解析 Length
  ↓
校验 Checksum
  ↓
检查帧尾 3E 3E
  ↓
根据 Channel 分发
```

---

## 14.1 Channel = 0x0006

```text
如果 Payload = 01 00：
    暂停主动上传

如果 Payload = 01 01：
    恢复主动上传
```

---

## 14.2 Channel = 0x0007

```text
读取 CMD
读取 COUNT

package_id = CMD >> 4
cmd_type = CMD & 0x0F
```

应满足：

```text
cmd_type = 0x03
```

然后根据 package_id 处理：

```text
package_id = 0：
    解析 meta
    记录当前 part_id
    清空当前 part 的临时缓存

package_id = 1：
    写入当前 part 的 area_cx
    检查 COUNT == area_num

package_id = 2：
    写入当前 part 的 area_cy
    检查 COUNT == area_num

package_id = 3：
    写入当前 part 的 force_grid
    检查 COUNT == force_num

package_id = 4：
    写入当前 part 的 sensor_table
    检查 COUNT == area_num × force_num
    如果 meta、area_cx、area_cy、force_grid、sensor_table 都收到：
        加载 LUT
```

---

## 14.3 下位机伪代码示例

```c
void on_lut_packet(uint8_t cmd, uint8_t count, const float *data)
{
    uint8_t package_id = cmd >> 4;
    uint8_t cmd_type = cmd & 0x0F;

    if (cmd_type != 0x03) {
        return;
    }

    switch (package_id) {
    case 0:
        current_part_id = (uint8_t)data[0];
        sensor_rows = (uint16_t)data[1];
        sensor_cols = (uint16_t)data[2];
        area_num = (uint16_t)data[3];
        force_num = (uint16_t)data[4];
        has_shape_feature = (uint8_t)data[5];

        clear_lut_cache(current_part_id);
        mark_meta_received(current_part_id);
        break;

    case 1:
        if (count != area_num) {
            report_error();
            return;
        }
        copy_area_cx(current_part_id, data, count);
        break;

    case 2:
        if (count != area_num) {
            report_error();
            return;
        }
        copy_area_cy(current_part_id, data, count);
        break;

    case 3:
        if (count != force_num) {
            report_error();
            return;
        }
        copy_force_grid(current_part_id, data, count);
        break;

    case 4:
        if (count != area_num * force_num) {
            report_error();
            return;
        }
        copy_sensor_table(current_part_id, data, count);

        if (is_lut_ready(current_part_id)) {
            force_lut_load_area_table_for_part_ex(
                current_part_id,
                sensor_rows,
                sensor_cols,
                area_num,
                force_num,
                area_cx,
                area_cy,
                force_grid,
                sensor_table
            );
        }
        break;

    default:
        report_error();
        break;
    }
}
```

---

# 15. 完整性检查规则

下位机至少应检查以下内容。

## 15.1 meta 检查

```text
COUNT 必须等于 6
part_id 必须是合法部位 ID
sensor_rows > 0
sensor_cols > 0
area_num > 0
force_num > 0
```

合法部位 ID 例如：

```text
0x11, 0x21, 0x31, 0x41, 0x51, 0x61
```

---

## 15.2 area_cx / area_cy 检查

```text
COUNT == area_num
```

---

## 15.3 force_grid 检查

```text
COUNT == force_num
force_grid 应该单调递增
```

---

## 15.4 sensor_table 检查

```text
COUNT == area_num × force_num
```

并且：

```text
area_num × force_num <= 255
```

因为当前 COUNT 是 1 byte。

---

# 16. 当前协议限制

## 16.1 COUNT 最大 255

当前数据帧里 `COUNT` 是 1 byte，所以单帧最多传：

```text
255 个 float
```

由于当前不做分包，因此必须保证：

```text
area_num × force_num <= 255
```

推荐配置：

```c
#define FORCE_LUT_MAX_AREA_NUM   20u
#define FORCE_LUT_MAX_FORCE_NUM  12u
```

最大：

```text
20 × 12 = 240
```

不会超过协议限制。

---

## 16.2 暂不支持分包

当前协议没有 offset 字段，所以一个 `sensor_table` 必须一帧发完。

如果未来需要：

```text
area_num × force_num > 255
```

则需要扩展协议，增加：

```text
offset
total_count
segment_index
segment_count
```

否则下位机无法拼接大表。

---

## 16.3 package_id=4 作为完成标志

当前没有额外 commit 包。

建议：

```text
收到 package_id=4 的 sensor_table 后，如果 0~3 都已收到，则认为当前部位 LUT 完成。
```

---

# 17. 示例数据整体结论

本文示例数据实际包含：

```text
1. 暂停主动上传控制帧
2. 部位 0x11 的完整 LUT
3. 部位 0x61 的完整 LUT
4. 恢复主动上传控制帧
```

未出现以下部位：

```text
0x21
0x31
0x41
0x51
```

通常原因是：

```text
这些部位没有找到对应 CSV 文件，所以上位机跳过下发。
```

部位 0x11 完整性检查：

```text
area_num = 3
force_num = 12
sensor_table_count = 36
3 × 12 = 36
结果：通过
```

部位 0x61 完整性检查：

```text
area_num = 5
force_num = 12
sensor_table_count = 60
5 × 12 = 60
结果：通过
```

---

# 18. 最终总结

当前 LUT 标定下发协议可以总结为：

```text
外层：
3C 3C | Channel | Length | Payload | Checksum | 3E 3E

控制通道：
Channel = 0x0006
01 00 = 暂停主动上传
01 01 = 恢复主动上传

LUT 下发通道：
Channel = 0x0007
Payload = CMD | COUNT | FLOAT_DATA[COUNT]

CMD:
0x03 = meta
0x13 = area_cx
0x23 = area_cy
0x33 = force_grid
0x43 = sensor_table
```

一个部位完整下发顺序：

```text
0x03 meta
0x13 area_cx
0x23 area_cy
0x33 force_grid
0x43 sensor_table
```

`sensor_table` 必须按 `[area][force]` 展开：

```c
index = area_index * force_num + force_index;
```

当前版本不分包，因此必须满足：

```text
area_num × force_num <= 255
```

如果 MCU 端暂时只使用基础 LUT，建议 `has_shape_feature` 固定为 `0`。
