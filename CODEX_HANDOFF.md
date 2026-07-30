# 循迹小车工程交接文档

更新时间：2026-07-30  
工程目录：`C:\Users\mateb\Desktop\ccs_project\ccs-project`  
当前分支：`main`  
生成交接文档前的源码基线：`a88cc60 (7.29_23:10)`

## 1. 用户当前目标与修改边界

用户只要求处理小车循迹、按键任务状态机、计时显示以及与循迹直接相关的参数，不要随意改动滚球控制、视觉等其他功能。

三个按键任务：

- KEY1：从A点出发，正常速度行驶一圈。
- KEY2：从A点行驶到B点。
- KEY3：从A点出发，以KEY1的70%速度慢速行驶一圈。

OLED目前只显示：

- `AB:0/1/2`
- `CD:0/1/2`

含义：

- `0`：该路段尚未切换。
- `1`：CH567红外条件触发切换。
- `2`：编码器距离保护触发切换。

七位数码管从按键启动开始计时，任务停车时停止并保留最终时间。

## 2. 当前状态机

主要状态机文件：`code/app/task.c`

路线阶段：

1. `ROUTE_AB_STRAIGHT`
2. `ROUTE_BC_ARC`
3. `ROUTE_CD_STRAIGHT`
4. `ROUTE_DA_ARC`

### AB直线切换到BC弧线

- 前90cm不累计红外转弯特征。
- 90cm后，在最近7帧内累计检测CH5、CH6、CH7。
- 三个通道可出现在不同帧，只要最近7帧按位或后覆盖CH567，就记录 `AB:1` 并进入BC。
- 如果红外未触发，AB距离达到220cm时记录 `AB:2` 并强制进入BC。

相关参数：

```c
TASK_AB_TURN_ARM_DISTANCE_CM = 90.0f
TASK_AB_ARC_PREP_DISTANCE_CM = 220.0f
```

CH567掩码：

```c
required_mask = 0x70U; /* CH5 | CH6 | CH7 */
```

### BC弧线切换到CD直线

- BC使用右弧线循迹。
- 本段累计MPU偏航达到180°后切换到CD直线。
- 进入CD时会重置编码器，因此CD距离是分段距离，不会继续使用AB的累计值。

### CD直线切换到DA弧线

- 前60cm不累计红外转弯特征。
- 60cm后，同样在最近7帧内累计CH567。
- 红外触发时记录 `CD:1`。
- 红外未触发、CD距离达到170cm时记录 `CD:2`。

相关参数：

```c
TASK_CD_TURN_ARM_DISTANCE_CM = 60.0f
TASK_CD_ARC_PREP_DISTANCE_CM = 170.0f
```

KEY1在CD距离达到125cm后会把基础速度降低5。KEY3已经改为不执行这次额外降速，因为KEY3本身已是70%慢速。

### DA弧线与整圈结束

- KEY1累计总偏航达到340°后停车。
- KEY3累计总偏航达到340°后停车。
- 当前没有“340°后左侧电机额外运行0.5秒”的代码，该额外动作已删除。

## 3. 三个按键当前参数

### KEY1

- 速度比例：100%
- 直线P：6.5
- 直线D：0.04
- 弧线P：14.4
- 弧线D：0.2
- 入弧前400ms额外右转辅助：10
- 超时保护：30秒
- 结束条件：总累计偏航绝对值达到340°

最近实车结论：KEY1基本没有问题。

### KEY2

- 从A到B。
- 检测B点特征或直线距离达到220cm时停车。
- 超时保护：8秒。

近期主要调试集中在KEY1/KEY3，没有重新验证KEY2。

### KEY3

- 速度比例：70%（刚从60%提高到70%）
- 直线P：3.0（刚从2.0提高到3.0）
- 直线D：0.02
- 弧线P：14.4
- 弧线D：0.2
- 入弧前400ms额外右转辅助：5
- CD不再额外降低5
- 丢线搜索使用原始 `3/8`，不再乘70%速度比例
- 超时保护：40秒
- 结束条件：总累计偏航绝对值达到340°

重要：以上KEY3优化刚完成并编译通过，尚未收到用户的最新实车复测结果。

优化前的实车现象是：

- OLED显示 `AB:2 CD:0`。
- 小车停在CD末端、接近DA弧线的位置。
- 当时处于丢线状态。

已定位的旧版本原因：

- KEY3原速度只有60%。
- CD到125cm后又降低5，速度比例约降到44%。
- 丢线搜索 `3/8` 再乘44%后整数截断为约 `1/3`，电机几乎没有能力重新找线。
- KEY3直线P原来只有2.0，纠偏较弱。
- 最后可能达到40秒超时并正式停车。

下一步应先实车复测当前70%版本，不要立即继续改参数。

## 4. 直线切弧当前行为

此前曾有“进入弧线后先普通循迹1秒，再加入强制右转”的逻辑，已取消。

当前：

```c
ARC_LINE_ONLY_MS = 0U
```

因此AB→BC、CD→DA状态切换后立即执行弧线循迹和右转前馈，不再运行1秒普通直线循迹。

右弧线方向：

```c
TASK_ARC_MIRROR_DIR = -1.0f
```

右弧线基础前馈：

```c
BASE_SPEED_ARC = 30
ARC_CURVE_BIAS = 20
```

进入右弧的前400ms还会增加额外右转量：

- KEY1额外量：10
- KEY3额外量：5

右弧时该前馈表现为左轮命令增加、右轮命令减小。KEY3已经单独减小，不影响目前基本正常的KEY1。

## 5. 当前关键参数汇总

`code/app/task.c`：

```c
TASK_ARC_YAW_DEG              = 180.0f
TASK_KEY1_TARGET_YAW_DEG      = 340.0f
TASK_KEY3_TARGET_YAW_DEG      = 340.0f

TASK_AB_TURN_ARM_DISTANCE_CM  = 90.0f
TASK_CD_TURN_ARM_DISTANCE_CM  = 60.0f
TASK_AB_ARC_PREP_DISTANCE_CM  = 220.0f
TASK_CD_ARC_PREP_DISTANCE_CM  = 170.0f
TASK_CD_SLOWDOWN_DISTANCE_CM  = 125.0f

TASK_NORMAL_SPEED_PERCENT     = 100U
TASK_SLOW_SPEED_PERCENT       = 70U
TASK_CD_SPEED_REDUCTION       = 5U
```

`code/inc/app_config.h`：

```c
BASE_SPEED_STRAIGHT           = 30
SEARCH_SPEED_LOW              = 3
SEARCH_SPEED_HIGH             = 8

KP_LINE_STRAIGHT              = 6.5f
KD_LINE_STRAIGHT              = 0.04f
KP_LINE_STRAIGHT_KEY3         = 3.0f
KD_LINE_STRAIGHT_KEY3         = 0.02f

BASE_SPEED_ARC                = 30
ARC_CURVE_BIAS                = 20
KP_LINE_ARC                   = 14.4f
KD_LINE_ARC                   = 0.2f
KP_LINE_ARC_KEY3              = 14.4f
KD_LINE_ARC_KEY3              = 0.2f

ARC_ENTER_ASSIST_MS           = 400U
ARC_ENTER_TURN_BIAS           = 10
ARC_ENTER_TURN_BIAS_KEY3      = 5
ARC_LINE_ONLY_MS              = 0U
```

## 6. 已确认的诊断结论

### CH567逻辑是有效的

CD曾稳定显示 `CD:1` 并顺利过弯，因此：

- 0x70掩码能够工作。
- 最近7帧累计逻辑能够工作。
- OLED切换来源显示能够工作。

工程注释假定 `bit0=X1最左侧、bit7=X8最右侧`。静态代码不能证明硬件接线是否镜像，但CD成功触发CH567说明当前CH567至少能识别实际弯道特征。

### AB显示2不一定是通道错误

从A点启动时曾出现 `AB:2`；把车放在AB中间再启动时出现 `AB:1`。

这说明：

- 每次启动编码器都从当前位置清零。
- 从A启动时，编码器兜底可能在CH567七帧累计完成前抢先触发。
- 从AB中间启动时，到B的编码器距离不足以触发兜底，红外有足够时间完成累计。

因此AB编码器保护已从180cm提高到220cm。当前版本仍需结合下一次实车结果判断KEY3是否还显示 `AB:2`。

### 弧线末端猛转的旧问题

旧问题由 `ARC_LINE_ONLY_MS=1000` 引起：

- 进入弧线后继续普通循迹1秒。
- 丢线后低速搜索。
- 1秒结束才突然加入右转前馈。

现在该值已改为0，直线会直接切到弧线。

## 7. 重要文件

- `code/app/task.c`
  - 三按键任务状态机
  - AB/BC/CD/DA阶段切换
  - CH567七帧累计
  - 编码器距离保护
  - OLED的AB/CD来源显示
  - 超时和340°停车

- `code/app/app_task.c`
  - 直线循迹执行
  - 丢线搜索
  - 弧线前馈和弧线循迹
  - KEY3丢线搜索不缩放
  - KEY1/KEY3不同的入弧辅助量

- `code/app/pid.c`
  - KEY1/KEY3 PID配置选择
  - `pid_is_key3_profile()`供弧线和搜线逻辑判断当前配置

- `code/inc/app_config.h`
  - 直线、弧线、速度环参数
  - KEY1/KEY3独立参数

- `code/drivers/line_sensor.c`
  - 4051八路读取
  - `bit0=CH1 ... bit7=CH8`

- `code/drivers/encoder.c`
  - 编码器计数和距离换算
  - `encoder_reset()`会在每次阶段准备时调用

- `code/drivers/number.c`
  - 七位数码管计时显示

## 8. 编译与警告

已使用以下命令成功完整编译：

```powershell
E:\download\ccs\utils\bin\gmake.exe -C Debug -j4 all
```

当前只有两个已有警告：

```text
unused function 'detect_left_turn_feature'
unused function 'detect_line_lost'
```

它们位于 `code/app/task.c`，与当前运行问题无直接关系，不要为了消除警告顺手改动状态机。

## 9. 下一个聊天的建议工作方式

1. 先完整读取本文件。
2. 再读取 `task.c`、`app_task.c`、`pid.c`、`app_config.h` 的当前内容。
3. 先询问或读取用户对最新KEY3 70%版本的实车结果。
4. 不要恢复历史版本，不要覆盖用户已有修改。
5. 每次只改用户明确要求的参数或逻辑。
6. 修改后运行完整编译。
7. 若KEY3仍在CD末端丢线：
   - 先看OLED最终是否仍为 `CD:0`。
   - 看数码管是否停在40秒，确认是否超时。
   - 再判断是否需要提高搜线速度或KEY3直线P。
8. 若KEY3已经能显示 `CD:1` 但转弯过猛：
   - 优先微调 `ARC_ENTER_TURN_BIAS_KEY3`。
   - 不要改KEY1的 `ARC_ENTER_TURN_BIAS`。
9. 若AB仍显示2但实际过弯正常，不要仅为了显示1盲目继续提高兜底距离；先确认CH567原始模式和实际切换位置。

