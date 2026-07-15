# RT1064 planner 单板测速

测速只在 Keil 目标 `freertos_hello flexspi_nor_debug` 中启用。该目标上电后会在
启动 FreeRTOS、底盘和视觉串口之前运行全部测试，因此不需要连接视觉端或其他
串口设备，也不会把串口发送时间计入 planner。

## 使用方法

1. 在 Keil 中选择 `freertos_hello flexspi_nor_debug`。
2. 编译并下载到 RT1064。
3. 打开 Debug，添加 Watch 表达式 `g_plannerBenchmarkReport`。
4. 全速运行。板载 LED 常亮表示 9 张地图已经全部测试结束。
5. 暂停运行并展开 `g_plannerBenchmarkReport.results` 查看结果。

不要使用该 benchmark 目标控制真实小车，因为测试结束后程序会停在主循环，
不会启动 FreeRTOS。正常小车固件继续使用 release 目标；其他五个 Keil 目标没有
启用 benchmark。

## 结果字段

- `state=2`：全部测试通过；`state=3`：至少一个测试失败。
- `systemCoreClockHz`：本次计时使用的 RT1064 核心时钟。
- `setupCycles`：创建地图、位姿和标签状态的周期数，不计入规划耗时。
- `beginCycles`：`planner_job_begin()` 的周期数。
- `stepCycles`：所有 `planner_job_step(1)` 的累计周期数。
- `totalCycles`：`beginCycles + stepCycles`。
- `totalUs`：planner 的总 CPU 时间，单位微秒。
- `stepCalls`：分片调用次数。
- `maxStepUs`：最慢一次 `planner_job_step(1)` 的时间。
- `planSteps`：生成计划的步骤数。
- `passed`：规划状态、分片次数和计划步数是否都与主机基准一致。

测试使用 `map_file` 目录中的全部 9 张 16x12 地图。每张地图的三个箱子按扫描
顺序赋标签 0、1、2，目标标签采用六种排列中“可解且搜索调用次数最多”的排列，
因此这里测的是确定且可重复的压力用例，不是随机标签结果。

## 计时边界

计时使用 Cortex-M7 DWT `CYCCNT`。每一次函数调用独立计算无符号周期差，再累计
到 64 位总数。只要单次 `planner_job_begin()` 或 `planner_job_step(1)` 的耗时小于
2^32 个周期，整组测试超过一次回绕周期也不会破坏总时间。计时区间内不执行
`printf`、串口发送、LED 翻转或动态内存分配。
