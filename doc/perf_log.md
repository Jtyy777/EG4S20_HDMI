# 性能迭代记录

> 每改一处 → 综合 → P&R → **记一行** → 上板回归 → git commit（说明里带前后数字）。
> 这张表就是评审要的"数据支撑"。

---

## 一、度量口径（每次必须一致，否则数字没有可比性）

| 指标 | 从哪来 | 说明 |
|---|---|---|
| SWNS / STNS | `_Runs/<run>/final_timing.rpt` 头部 | 全体最差 / 总负 slack |
| 各域周期与最高频 | 同一报告的 `Clock Summary` 表，`R-Freq` 列 | 域内实测 Fmax |
| LUT / REG / BRAM / DSP | `_Runs/<run>/info_signage_v1.0_phy.area` | 资源占用 |
| 上板功能 | 见第五节回归清单 | 功能不许因此退化 |

**P&R 策略必须固定**：`settings.cfg` 里 `start_step = opt_place`、`area_option = -packarea`。
换策略就是换变量，不能和 RTL 改动混在同一次对比里。

---

## 二、基线（2026-10-24 16:30，`phy_1`，tag `v1.0-func-freeze`）

```
SWNS -7.154ns   STNS -1286.681ns
LUT 10914/19600 (55.68%)   REG 6269 (31.98%)
BRAM 27/64 (42.19%)        DSP 22/29 (75.86%)  ← 最紧的资源
```

| 时钟域 | 约束频率 | 域内最差 slack | 最高可达频率 |
|---|---|---|---|
| video_clk（像素） | 25 MHz | **+6.027ns**（通过，33.693/40） | 29.7 MHz |
| sd_card_clk（读图/写侧） | 100 MHz | **−3.627ns**（13.627/10） | **73.4 MHz** |
| ext_mem_clk（SDRAM） | 125 MHz | 域内 −0.060ns（临界） | — |
| clk（晶振） | 50 MHz | +5.925ns | 95.5 MHz |
| hdmi_5x_clk（串行） | 125 MHz | +5.426ns | 388.5 MHz |

> 注：`sd_card_clk` 的 Fmax 曾误记为 47.0 MHz。报告 `sd_card_clk -> sd_card_clk` 组明确写
> `Min Period 13.627ns / Fmax 73.384MHz`，以报告为准。**这意味着该域只差 27% 的速度，
> 而不是差一倍** —— 这对后面判断"改代码划算还是降频划算"很关键。

---

## 三、61 条负 slack 的精确分类（按起止时钟分组，2026-10-25 重算）

| # | 发射 → 捕获 | 条数 | 最差 | 预算 | 延时(cell/net) | 级数 | 路径扇出 | 性质 |
|---|---|---|---|---|---|---|---|---|
| 1 | `sd_card_clk` → `sd_card_clk` | 10 | −3.627 | 10 ns | 13.18 (49%/51%) | 9~12 | 43 | **真·域内逻辑太长** |
| 2 | `clk` → `video_clk` | 10 | −7.154 | 20 ns | 25.06 (56%/44%) | 21~23 | 28 | **真·跨域无握手** |
| 3 | `sd_card_clk` → `video_clk` | 10 | −4.031 | 10 ns | 11.94 (40%/60%) | 4,12,13,14 | 15 | 真·CDC 缺同步器 |
| 4 | `sys_pll_m0` → `ext_mem_clk` | 10 | −6.699 | 4 ns | 6.45 (100%/0%) | **0** | 0 | **假·SDRAM DQ 硬核 IO** |
| 5 | `ext_mem_clk` → `sys_pll_m0` | 10 | −2.799 | 4 ns | 10.36 (79%/21%) | **1** | 34 | **假·SDRAM DQ 硬核 IO** |
| 6 | `ext_mem_clk` → `sd_card_clk` | 10 | −2.257 | **2 ns** | 4.02 (30%/70%) | 2~3 | 38 | **假·预算被算成 2ns** |
| 7 | `ext_mem_clk` → `ext_mem_clk` | 1 | −0.060 | 8 ns | 7.71 | 8 | 4 | 临界 |

### 三个必须看清的事实

**① 61 条里有 40 条根本不用改代码（第 4/5/6 组）**

- 第 4/5 组共 20 条，逻辑级数 **0 和 1**，`cell` 延时占 100% 和 79% —— 这就是 SDRAM 的 `dq` 双向数据线。它走硬核 IO 原语，报告里那 6.45ns / 10.36ns 是工具给硬核 IO 填的默认延时模型，**不管你代码怎么写都不会变**。
- 第 6 组的预算只有 **2.0 ns**：`ext_mem_clk`(125MHz, 8ns) 与 `sd_card_clk`(100MHz, 10ns) 的公共周期是 40ns，工具取了两个边沿之间最窄的那 2ns。而这 10 条路径只有 **2~3 级逻辑**，是 `frame_fifo` 的指针/水位跨域 —— 设计上靠异步 FIFO 交接，本来就不需要在 2ns 内跑完。

**这 40 条的正确处理是补约束，不是加流水线。** 对着它们改 RTL 等于白干。

**② `max_fanout` 这条参数对最差路径无效（原计划 E1 判断错误）**

报告里那个 `fo=119` 是**时钟网** `clk_dup_1`（50MHz 送 PLL 的参考时钟），不是数据路径。六组最差路径的**路径扇出只有 15~43**。把 `max_fanout` 设成 200 完全够不着它们；要生效得设到 15 以下，那会全局强拆网、面积暴涨，得不偿失。

**③ 第 1 组 net 延时占 51%，这才是布局阶段能改善的部分**

`cell 6.50ns + net 6.67ns`。逻辑级数 12 级里有 **7 级是 ADDER**（`lt8` 比较器被综合成进位链），这部分只能改 RTL；但那 6.67ns 的走线延时可以用 `opt_timing ultra` 让布局把相关逻辑收拢，通常能捞回 1~2ns。

### 结论：真正要动的只有 21 条

| 组 | 条数 | 怎么处理 | 属于哪一轮 |
|---|---|---|---|
| 第 1 组 域内 12 级 | 10 | 拆 ADDER 链 + 打拍（改 RTL）；布局收紧（P&R 参数） | R1 + R0 |
| 第 2 组 50→25MHz 23 级 | 10 | `contrast_q8` 加两级同步 + 打拍 | R2 |
| 第 3 组 100→25MHz 13 级 | 10 | `img_count`/`img_index` 加同步器 | R2 |
| 第 4/5/6 组 | 30 | 补 IO 延时约束 + 开 asynchronous | R2（零 RTL） |

**先做 R0 的意义**：把「布局能白捡多少」这个基线量出来。等 R2 改完约束、报告只剩第 1~3 组时，才知道哪些是布局的功、哪些是约束的功。

---

## 四、迭代计划（每轮单独 commit）

> **版本对应关系（容易混淆，先看清）**：
> git 历史里 **R0 与 R1 之间没有提交**，但不代表中间没做事——
> `ac3a614`（R0，14:18）只含 `perf_log.md` + `E0_baseline.rpt` + `bit`；
> 之后的 **E1 实验（14:36）只改了 `.al` 三行参数，当时没有提交**，跑完直接用同一份工作区接着做 R1，
> 于是 E1 的改动一直到 15:02 才随 R1 一起进库（`b94a4df`）。E1 因此在 git 里**从来没有独立版本**。
> 回滚 R1 后，E1 的参数与报告被单独提交为 **`8389897`**，这才等价于「E1 版本」的第一次入库。

### R0 策略扫描（零代码改动）

#### R0.0 出厂默认值存档（2026-10-25 实测 `report_param -stage place -detail`）

```
effort          medium   |  hfn_opt_effort  medium  |  max_fanout  999999
opt_timing      medium   |  post_place_opt  auto    |  pr_strategy 1
relaxation      1.00
```

**全部为默认值**，说明此前从未调过 P&R 参数，这是干净的起点。
`max_fanout = 999999` 等于**完全不限扇出**，这是最该动的一项。

#### 参数归属：两类参数影响的阶段不同，别混

| 阶段 | 参数 | 改完要跑什么 | 耗时 |
|---|---|---|---|
| `place` | `opt_timing` `max_fanout` `pr_strategy` `post_place_opt` `pack_mode` | **只重跑 Place & Route** | 15~20 min |
| `flow` | `effort` `hfn_opt_effort` `logic_level_opt` `speedup_effort` `enable_seed` | **要重跑综合**（`reset_runs syn_1`） | 35~45 min |

命令一律是 `set_param -run <run> <stage> <参数> <值>`，注意 `-stage` 只用在 `report_param` 上。

#### 实验顺序（一次只改一行，跑完立刻记一行）

**第 1 批：只动 place / route，不用重跑综合**

| 实验 | 命令 | 针对什么 | 预期 |
|---|---|---|---|
| E1 | `set_param -run phy_1 place opt_timing ultra` | 第 1 组 net 占 51% 的走线延时 | **首选**，1~2ns |
| E2 | `set_param -run phy_1 place post_place_opt ultra` | 布局后二次优化 | 中等，0.3~1ns |
| E3 | `set_param -run phy_1 route post_route_opt ultra` | 布线后优化（走 `route` 阶段） | 中等，0.3~1ns |
| E4 | `set_param -run phy_1 place pr_strategy 2`（依次 3、4…） | 布局算法变体 | 随机性大，靠多试 |
| E5 | E1~E4 中最好的组合 | — | 叠加，可能 1.5~2.5ns |

**第 2 批：动 flow，须重跑综合**

| 实验 | 命令 | 针对什么 |
|---|---|---|
| E6 | `set_param -run syn_1 flow logic_level_opt on` | 自动压逻辑级数，正对 12/21/23 级病灶 |
| E7 | `set_param -run syn_1 flow effort high` | 全局力度，可能涨面积 |

**不建议动 `max_fanout`**：六组最差路径的实际扇出只有 15~43（报告里 `fo=119` 是时钟网 `clk_dup_1`，不是数据路径）。要碰到它们得设到 15 以下，全局强拆网、面积暴涨，收益不成比例。真想试就放最后，设 `32` 看一眼即可。

#### E1 实验结果：`opt_timing = ultra`（2026-10-25 14:36，存 `E1_opt_timing_ultra.rpt`）

**设参确实生效**，三个独立证据：`.al` 多出 `<PlaceProperty><opt_timing>ultra</opt_timing>`；布局线长 576195 → 565889（−1.8%）；布局阶段 Setup WNS −6.643 → −5.643（+1.0ns）、FMAX 68.292 → 73.298MHz。

| 指标 | E0 基线 | E1 ultra | 变化 |
|---|---|---|---|
| 全局 SWNS | −7.154 | −6.699 | **+0.455** |
| 全局 STNS | −1286.681 | −1259.552 | +27.13 |
| `sd_card_clk` 域内 WNS | −3.627 | −3.736 | **−0.109** |
| `sd_card_clk` Fmax | 73.38 | 72.80 | −0.58 |
| `clk → video_clk` WNS | −7.154 | −6.195 | +0.959 |
| `sd_card_clk → video_clk` WNS | −4.031 | −3.703 | +0.328 |
| `video_clk` 域内 WNS | +6.027 | +4.339 | −1.688 |
| `clk → clk` WNS | +9.533 | +8.304 | −1.229 |
| LUT / REG / DSP / BRAM | 10914 / 6269 / 22 / 27 | 10920 / 6259 / 22 / 27 | +6 / −10 / 0 / 0 |

**结论三条**

1. **全局 +0.455ns 是"最差那条路径换组"，不是整体改善。** 变好的只有 `clk→video_clk`(+0.959)、`sd_card_clk→video_clk`(+0.328)、`ext_mem→sys_pll`(+0.844)；同一轮里 `clk→clk`(−1.229)、`video_clk` 域内(−1.688)、`clk→sd_card_clk`(−1.031) 都在变差。这是布局的**重新分布**，属噪声级别。
2. **要修的第 1 组没修动。** 最差路径拓扑与 E0 完全相同：`bmp_read/img_h → u_img_scaler/gx_reg[*].ce`（同时驱动 `mult1x_syn_3.ceb`），13 级逻辑、其中 **10 级是 ADDER**，cell 7.155ns(53%) + net 6.301ns(47%)。
3. **对比 E0（12 级，cell 6.50 / net 6.67）与 E1（13 级，cell 7.16 / net 6.30）：布线少走了 0.37ns，综合却多串了一级 ADDER，一进一出正好抵消。** 说明瓶颈在**逻辑级数**，布局优化帮不上忙。

**处置**：参数**保留 ultra 不回退**（回退要再花 20 分钟，而下一轮改完 RTL 还要重跑，收益为零）。从 E1 起所有轮次都在 `opt_timing=ultra` 上比较。R0 的结论是：**布局参数这条路最多捞 0.3~1ns（`post_place_opt`/`pr_strategy`），而缺口是 3.74ns —— 直接进 R1 改 RTL**，多 seed 留到最后收尾。

#### 每次必做两件事

1. `report_param -stage place -detail`（或 `-stage flow`）确认参数真的进去了 —— **TD 会静默忽略拼错的参数名和非法取值**。
2. **跑完 P&R 立刻把 `phy_1/final_timing.rpt` 复制到 `doc/timing_rpt/E<n>_<说明>.rpt`** —— 下一次 P&R 会把原文件覆盖掉，历史数据就没了。基线已存为 `E0_baseline.rpt`。

#### 官方给出的参数取值范围（`SWUG105` 表 4-2）

| 参数 | 合法取值 | 默认 |
|---|---|---|
| `place max_fanout` | 9 ~ 999999（整数） | 999999 |
| `place opt_timing` | off / low / medium / high / ultra | medium |
| `place pr_strategy` | PH1 器件：1~11 | 1 |
| `place post_place_opt` | auto / medium / high / ultra | auto |
| `place pack_mode` | auto / lower_pin_density / adaptive_pin_density / more_lut_pair | auto |
| `flow effort` | medium / high | medium |
| `flow hfn_opt_effort` | low / medium / high | medium |
| `flow logic_level_opt` | off / on | off |
| `flow speedup_effort` | 0 / 1 / 2 / 3 / 4 / 5 | 0 |
| `flow hfn_to_clock` | ≤ 1000000000 | 1000000000 |
| `route effort` | normal / routability | normal |
| `route post_route_opt` | auto / medium / high / ultra / off | auto |

### R0+ 多 seed（TD 自带，不用手写脚本）

同一个设计、同一份综合网表，只换布局布线的随机种子，WNS 通常能差 1~3ns：

```tcl
set_param -run phy_1 flow enable_seed on
set_seed_param -num 6 -init 1 -step 10 -jobs 3 -save 1
```

`-num 6` 跑 6 个种子，`-jobs 3` 同时跑 3 个，`-save 1` 只保留最好的那个。
6 个种子约 40 分钟。结果在 `phy_1/seed_*/`，最好的会被 `save_best_bits` 收进 `best_result/`。

想提前砍掉差的种子（省时间）：

```tcl
set_seed_param -num 6 -init 1 -step 10 -jobs 3 -save 1 -filter on -rwns -1.0
```

`-rwns -1.0` = 布线阶段 WNS 差于 −1ns 的种子直接放弃。

### R1 修唯一真违例：`img_scaler` 的行选择逻辑（`sd_card_clk` 域）

> **⚠️ 本节代码已回滚（2026-10-25 15:11，commit `6204cb4` revert `b94a4df`）。**
> 结论与数据保留作记录，当前工程里**没有**这些改动。回滚原因见本节末尾「R1 回滚记录」。

- **病灶（E1 实测）**：`bmp_read/img_h` → `u_img_scaler/gx_reg[*].ce`、`mult1x_syn_3.ceb`，
  13 级逻辑（**ADDER×10** + LUT3×2 + LUT5×1），13.456ns = cell 7.155 + net 6.301，需回收 ≥3.74ns
- **病因**：`img_scaler.v:511~515` 三个 `wire` **每拍组合重算**，且 `src_h - 16'd1` 在链上现算
  两次（`sy` 一次、`sy_hi` 一次），再 `+1` 去和 `done_rows` 比较；这条链直接驱动 `gx` 与
  DSP 的时钟使能，又和状态机条件（`p_ok` / `g_fin` / `yo` 的两个范围比较）并成同一个 13 级锥
- **手法（只加寄存器，不改算法）**：
  1. `src_h_m1 <= src_h - 16'd1` 单独打一拍（`src_h` 整帧不变，纯提一级减法出路径）
  2. `yoff_end <= yoff + out_h` 单独打一拍（同上，去掉 `yo >= yoff+out_h` 里的加法）
  3. `sy_r / sy_hi_r / row_avail_r` 由 `yacc` 打一拍产生，`G_IDLE` 改用寄存版本
  4. 于是 `gx.ce` 只依赖 `p_ok / g_fin / row_avail_r` 三个寄存量 + 一级比较
- **代价**：每输出行多 1~2 拍（10~20ns），640 行合计 <13µs，相对 2 秒的读图时间可忽略
- **目标**：`sd_card_clk` 域 WNS ≥ 0（Fmax ≥ 100MHz），资源变化控制在 ±30 LUT 内

#### R1 实施记录（2026-10-25 14:5x，`rtl/img_scaler.v`）

改动 54 增 8 删，全部是加寄存器与改引用，**算法一行没动**：

| 位置 | 改动 |
|---|---|
| 新增 `src_h_m1` | `src_h-1` 打一拍，移出关键路径 |
| 新增 `yoff_end` | `yoff+out_h` 打一拍，替掉 `(yo >= (yoff+out_h))` 里的加法 |
| 新增 `sy_s1` | 第 1 级：`clamp(yacc)` |
| 新增 `sy_hi_s2` | 第 2 级：`clamp(sy_s1+1)` |
| 新增 `row_avail_r` | 第 2 级：两条源行是否到齐 |
| 新增 `same_row` | `sy_hi_s2 == sy_s1`，等价原 `(sy_hi == sy)` |
| 新增 `ysettle` 计数 | `yacc` 更新后等 3 拍再放行 G_IDLE，保证派生量与 `yacc` 对齐 |

**为什么需要 `ysettle`**：两级流水意味着 `sy_s1` 落后 `yacc` 1 拍、`sy_hi_s2`/`row_avail_r`
落后 2 拍、`src_h_m1`（由 `src_h` 再打一拍）也落后 1~2 拍。若不等对齐就直接起新行，
会拿**上一行**的 `same_row` 去选 `b_cur/b_prev`，垂直插值选错缓冲。
代价是每行 3 拍（30ns），480 行共 14µs。

**新增延迟对帧率的核算**：生成一帧原约 9ms，现在 9ms + 14µs ≈ 9.014ms，
仍有 16.8ms 的显示帧时间兜着；第 470 行那条「填充不得追上生成」的速率条件余量 10 倍以上，不受影响。

**验证状态**：综合 + P&R 已跑（E2，见下）。上板回归见第五节清单，**尚未上板**。

#### R1 结果（E2，2026-10-25 14:58，报告存 `doc/timing_rpt/E2_r1_img_scaler_pipe.rpt`）

**结果：结构性目标达成，但净收益小于预期——因为后面藏着第二条同性质的链。**

| 指标 | E1 基线 | E2 改码后 | 变化 |
|---|---|---|---|
| `img_scaler` 相关负 slack 条数 | **8 条** | **0 条** | **−8（彻底清除）** |
| `sd_card_clk` 域内最差级数 | 13 级（ADDER×10） | 10 级（ADDER×2） | −3 级，ADDER −8 |
| 该路径 cell 延时 | 7.156 ns（53%） | 5.355 ns（40%） | **−1.80 ns** |
| 该路径 net 延时 | 6.301 ns（47%） | 7.920 ns（60%） | +1.62 ns（布局变散） |
| `sd_card_clk` WNS | −3.736 | **−3.555** | +0.181 |
| `sd_card_clk` Fmax | 72.80 MHz | 73.83 MHz | +1.03 |
| `sd_card_clk` 失败端点 | 226 | 213 | −13 |
| 全局 STNS | −1259.552 | **−1147.274** | **+112.3** |
| `video_clk` 域内 WNS | +4.339 | **+6.938** | +2.599 |
| LUT / DSP / BRAM | 10920 / 22 / 27 | 10958 / 22 / 27 | +38 / 0 / 0 |

**为什么 WNS 只好了 0.18ns**：`img_scaler` 那条链确实被打断了（cell 少 1.8ns、ADDER 10→2），
但**最差路径换成了同域内的另一条**——`bmp_read` 的「FAT 簇号 → 物理扇区地址」换算：

```
起 sd_card_bmp_m0/bmp_read_m0/sectors_per_cluster_reg[2]
终 sd_card_bmp_m0/bmp_read_m0/sd_sec_read_addr_reg[28]
10 级 (LUT4×4 LUT5×3 ADDER×2 LUT3×1)   13.275ns = cell 5.355(40%) + net 7.920(60%)
```

而这条路径的 **net 占 60%**，说明它现在不是"逻辑太长"而是"走线太远"，纯靠布局救不回来，
必须同样拆流水。**这正是本轮的额外收获：证明瓶颈是成串出现的，一次只能揭开一个。**

至于 `video_clk` 域 +2.599ns 和 STNS +112ns：前者是布局重排的顺带收益（`audio_spectrum`
归一化链恰好被摆得更近），后者来自 213 个端点不再超时。**这两项都不该记在 R1 头上**，
R1 的功劳只算「`img_scaler` 违例 8→0」和「cell −1.8ns」。

#### R1 回滚记录（2026-10-25 15:11，commit `6204cb4`）

**决定：撤销 R1，工程回到 E1 状态（`ac3a614` + `.al` 保留 `opt_timing ultra`）。**

回滚的操作与校验：

| 步骤 | 做法 |
|---|---|
| 代码回退 | `git revert b94a4df`（保留历史，随时可再取回） |
| 还原校验 | `git diff ac3a614 -- rtl/img_scaler.v` → **0 行差异**；`ysettle / sy_s1 / same_row` 全部消失 |
| 抢回归档 | revert 顺带删掉了 `doc/timing_rpt/` 三份报告（当时与代码同提交），已 `git checkout b94a4df --` 取回 |
| 保留 E1 成果 | `.al` 的 `<opt_timing>ultra</opt_timing>` 是 E1 的，不属于 R1，已恢复保留 |

**为什么回滚**：R1 的净收益太小，不值得为它承担上板风险——
- 域内 WNS 只从 −3.736 走到 **−3.555**（+0.181），Fmax 72.80 → 73.83，距 100 MHz 目标**仍差 26%**
- 最差路径换成 `bmp_read` 的簇号→扇区地址换算，`net` 占 60%，说明真正瓶颈从「逻辑太长」变成了「走线太远」
- 为 0.18ns 引入两级流水 + 3 拍 `ysettle` 对齐，出错症状**隐蔽**（不黑屏不花屏，只换图后头一两行颜色串），本机无仿真器，只能靠上板取证

**保留的判断性结论**（这些不受回滚影响，仍是后续工作的依据）：
1. 违例是**成串**出现的，修掉一条立刻顶出下一条；单条 RTL 优化无法拉平 3.7ns 的缺口
2. `img_scaler` 那条链确实是真病灶（cell 可降 1.8ns、ADDER 10→2），**若将来重做 R1b 仍应按此思路**
3. 缺口 3.7ns 量级下，「零散拆流水」性价比低；**要么一次拆透（R1+R1b 一起），要么直接降频 70MHz**

**如果以后要取回 R1 代码**：`git cherry-pick b94a4df`，或 `git checkout b94a4df -- src/user_source/hdl_source/rtl/img_scaler.v`。

#### ⚠️ bit 产物与 `ultra` 参数不一致（2026-10-25 15:15 核对发现）

回滚后 `best_result/info_signage_v1.0.bit` 的哈希是 `dcb61be`，即 **`ac3a614` 那一版的 bit**。
而 `ac3a614` 提交于 14:18，**E1 的 `ultra` 实验 14:36 才跑**——所以这个 bit 是 `ultra` 之前的布局产物。

三版 bit 哈希（路径 `src/td_project/info_signage_v1.0_Runs/best_result/info_signage_v1.0.bit`）：

| 版本 | bit 哈希 | 说明 |
|---|---|---|
| `1158076` 基线 | `8274067` | 10-24 封版那次 P&R |
| `ac3a614` R0 | `dcb61be` | **当前工作区就是这个**，早于 `ultra` |
| `b94a4df` R1/E2 | `f7199ec` | R1 代码 + `ultra` |

**E1 自己的 bit 从未入库，已丢失**：E1 跑完时覆盖了 `phy_1/`，紧接着 E2 又覆盖了一次
（`phy_1/info_signage_v1.0.bit` 现为 14:58 的 E2 产物）。E1 当时没提交，所以没有副本。

**结论**：现在烧进去能正常工作（源码就是 E1 的源码，功能没差），但**性能是 `ultra` 之前的水平**。
要拿到 `ultra` 的实际效果，必须重跑一次 P&R。**不要再把 `best_result` 的 bit 当作 `ultra` 的证据。**

**教训**：`best_result/` 是会被每次 P&R 覆盖的输出目录，它只在 commit 那一刻代表某个确定状态。
实验做完要立刻提交（或至少 `cp` 一份到 `doc/timing_rpt/` 旁边），否则一次覆盖就没了。

#### 15:20 重跑 P&R，bit 已对平（commit `8ca4952`，15:28:26）

重跑后逐项核对，与 E1 归档报告**完全一致**，说明 `ultra` 这个布局结果可复现：

| 核对项 | 结果 |
|---|---|
| `best_result` bit 哈希 | `dcb61be`（旧，`ultra` 前）→ **`7c19647`** |
| `_phy.ts` SWNS / STNS | `−7.154 / −1286.681` → **`−6.699 / −1259.552`** |
| `phy_1/final_timing.rpt` | `SWNS −6.699 / STNS −1259.552`，与 `E1_opt_timing_ultra.rpt` **同值同大小（1763266 字节）** |

至此「代码 = 基线、`.al` = `ultra`、`bit` = `ultra` 布局」三者对平，
`8ca4952` 就是完整的 E1 状态，可以拿去上板。

> `phy_1/final_timing.rpt` 下次 P&R 会被覆盖，但内容已与 `E1_opt_timing_ultra.rpt` 相同，无需重复归档。


### R1-备选 B：降 `sd_card_clk` 频率（比改 RTL 更快，但有代价）

`sd_card_clk` 目前 100 MHz，最差路径 13.627 ns → 实测 Fmax 73.4 MHz。若把该域降到 **70 MHz**（14.286 ns），违例**直接转正**，零 RTL 改动。

**为什么能降**：这个域只驱动 `sd_card_bmp` 这一块（SD 卡控制器 + `bmp_read` + `img_scaler`），不涉及时序敏感的显示通路。它的负荷极低——读一张图约 2 秒，期间有 2 亿个周期处理 200 万像素，利用率约 1%。当初取 100 MHz 只是为了让 SPI 高速档落在 25 MHz（`100 / ((0+2)*2)`）。

**代价（三处必须同步改，否则功能出错）**：

| 影响 | 位置 | 说明 |
|---|---|---|
| SPI 速率 | `SPI_HIGH_SPEED_DIV = 0` → `clk/((div+2)*2)` | 70 MHz → 17.5 MHz，仍在 SD 规范内，但读图时间 × 1.43 |
| OSD 秒数 | `sd_card_bmp` 参数 `CLK_FREQ_HZ = 100_000_000` | 切图耗时的 T/W/L 全部算错，必须改成实际频率 |
| 按键去抖 / 自动轮播 | 同上参数的 `DEBOUNCE_CYCLES` / `auto_tick` | 去抖时间、轮播间隔同样算错 |

改 PLL 只能在 TD 的 IP 配置界面改 `sys_pll` 的 clk0，**不能手改 `.ipc`**（会被重新生成覆盖）。

**优先级**：排在 R0 之后。若 R0 的 max_fanout 就能把 −3.627 拉到 −1.5 以内，R1 的 RTL 改动量会小很多；若 R0 毫无效果，降频是最省力的兜底。两条路互不冲突，可以叠加。

### R1b 拆 `bmp_read` 的簇号→扇区地址换算（E2 揭示的新瓶颈，`sd_card_clk` 域）

> **⚠️ 已挂起（随 R1 一并回滚）。** 下面记录的病灶位置与拆法仍然有效——
> 它是 E2 实测出的**真实最差路径**，与 R1 是否保留无关。若将来恢复优化，从这里开始。

- **病灶（E2 实测）**：`sectors_per_cluster_reg[2]` → `sd_sec_read_addr_reg[28]`，
  10 级（LUT4×4 LUT5×3 ADDER×2 LUT3×1），13.275ns = cell 5.355（40%）+ **net 7.920（60%）**
- **代码位置**：`SD/bmp_read.v:646`
  ```verilog
  sd_sec_read_addr <= part_start_eff + first_data_sector
                    + (({4'd0, fat_item_n[27:0]} - 32'd2) << spc_shift);
  ```
  同文件 487/490/505/536 行还在用 `* sectors_per_cluster` 做 32 位乘法（646 行已改成移位，不一致）
- **链有多长**：`sectors_per_cluster`（寄存器）→ 8 路 `case` 组合译码出 `spc_shift`（第 159 行
  `always @*`）→ 变量移位器 → 减 2 → 加 `part_start_eff` → 加 `first_data_sector`。
  **三个加法 + 一段变量移位 + 一段 case 译码堆在同一个时钟沿里。**
- **拆法（三步，都不改功能）**：
  1. `spc_shift` 改成**寄存器**：在 BPB 偏移 13 读回 `sectors_per_cluster` 的同一拍
     （第 304 行）顺便译码存起来。之后移位量直接来自触发器，砍掉 3 级 case 译码树。
  2. 新增 `data_base`（寄存器）= `part_start_eff + first_data_sector`，在 `ST_SCAN_BPB`
     结束时赋值（这两个量早于 FAT 链很久就确定了）。第 646 行的三个加法变一个。
  3. 若仍不够：新增 `clus_off`（寄存器）= `(fat_item_n-2) << spc_shift`，下一个状态
     `sd_sec_read_addr <= data_base + clus_off`。多 1 拍，只在**簇跳转**时出现
     （1MB 图 / 32KB 簇 = 32 次，共 320ns，相对 2 秒读图完全不可见）。
- **预期**：三步后 10 级 → 约 5 级，且加法从 3 个降到 1 个，cell 与 net 都会缩；
  目标 `sd_card_clk` WNS ≥ 0
- **风险**：`spc_shift` 改寄存器要确认它在 BPB 阶段之后**不再变化**（`sectors_per_cluster`
  只在偏移 13 写一次），以及 487~536 行的 `*` 换成 `<<` 时格式必须合法（FAT32 规范要求
  `SecPerClus` 为 2 的幂，646 行早已这么假设，两处一致）

### R2 收口约束（零 RTL 改动）
- SDRAM `DQ` 补 `set_input_delay` / `set_output_delay`（或按厂商模板 false_path）
- 第 2/4/5 组确认为异步的，开 `set_clock_groups -asynchronous`，**每条都要在 SDC 里写清理由**
- 目标：TNS 从 −1286ns 降到个位数，报告里只剩真问题

### R3 给像素域加余量
- 病灶：`audio_spectrum` 的 `bi → logq_max` 归一化链，25 级 33.693ns，只剩 6ns 余量
- 手法：对数/归一化拆成 2~3 拍流水
- 目标：域内最差 ≤ 20ns（Fmax ≥ 50MHz），为任何算法升级留空间

### R4（可选，画质）缩放抗混叠
- 缩小倍数 ≥2 时水平取 4 抽头加权，消除摩尔纹
- 代价：DSP 22 → 约 24（上限 29，还有 5 个余量）
- 必须先完成 R1，否则在已超时的路径上继续加重

### F1 切图耗时 OSD 归属修正（2026-11-03，功能项，不动时序关键路径）

**现象**：屏幕正放着第 N 张时，右上角 T/W/L 显示的是第 N+1 张的读数。

**根因**：`SD/sd_card_bmp.v` 里 `switch_latency` 把「发布」挂在 `commit = load_frame_done`
（整帧写完）。三缓冲预读把"读卡"和"上屏"两件事解耦了——第 N 张上屏后 50ms 就开始读
第 N+1 张，2.4s 读完，此时屏幕还在放第 N 张，commit 一到就翻转 `sw_upd_tgl`，
OSD 的数字被下一张覆盖。`S_WAIT` 里那个 300ms 超时兜底更是保证它**一定**提前发布：
预读场景下本张停留期内 `disp_busy` 根本不会起来，必然走超时分支。

| 时刻 | 发生什么 | 屏幕上是 | OSD 上（旧） |
|---|---|---|---|
| t0 | 第 N 张上屏 | N | N |
| t0+50ms | 开始读第 N+1 张 | N | N |
| t0+2.4s | 第 N+1 张整帧写完 → **发布** | N | **N+1** ← bug |
| t0+3s | 第 N+1 张上屏 | N+1 | N+1 |

**改法**（`SD/sd_card_bmp.v` +216/−106，`rtl/osd_overlay.v` 只改注释）

| 项 | 旧 | 新 |
|---|---|---|
| 观测点 | 3（`load_start` / `commit` / `disp_busy`） | 4（多一路 `swap = swap_now \| load_commit_now`） |
| 发布时刻 | 整帧写完 | **这一张上屏 → 它的转场落下** |
| 寄存器 | 单级状态机 | 两级：`m_*/p_*` 管读卡，`q_*` 管发布 |
| 兜底 | 写完起 300ms 无转场就发 | 上屏起 300ms 仍是兜底，另加转场 6s 强制收尾 |

**为什么必须拆两级**：预读让两个动作在时间上重叠——第 N 张的淡入（1.07s）还没走完，
第 N+1 张的 `load_start` 就已经进来了。共用一个状态机会把没发布完的第 N 张结果冲掉，
这正是旧代码那个 300ms 分支的由来。拆级后 `q_*` 只在 `swap` 时装载，与读卡阶段彻底无关。

**顺带补的口径漏洞**：按键命中预读时切图**不产生** `load_start`（帧早读好了），
继续用 `p_treq` 作基准会让 T 变成"上一张上屏到现在"，按 10 秒还会撞上 9.99 的饱和值。
现在这一路（`swap && req_pending`）T 取按键时刻、W 记 0。

**口径（2026-11-03 与用户确认，保持旧定义）**

| 量 | 定义 | 自动轮播 | KEY0 命中预读 | KEY0 无预读 |
|---|---|---|---|---|
| W | 请求 → 开始读卡 | 0.00 | 0.00（帧已就绪） | 静默窗长度 |
| L | 开始读卡 → 整帧写完 | 读卡耗时 | 该图读卡耗时 | 读卡耗时 |
| T | 请求 → 新图完全可见 | ≈4.1（含等停留） | ≈1.07（只算转场） | 按键→可见 |

> 自动轮播的"请求"取开始读卡时刻，而预读在上一张上屏后立刻启动，所以 T 天然含
> "等停留到点"那一段（1080P：0.05 静默窗 + 2.4 读卡 + 0.6 停留余量 + 1.07 转场）。
> **判断"3s 停留期内读不读得完"看 L，不是 T。**

**资源预估**：纯观测逻辑，新增 2 个 16 bit 比较器 + 约 40 bit 寄存器，预期 LUT +20~40，
不触碰任何 DSP / BRAM；不进 `sd_card_clk` 域的最差路径（原最差路径在 `bmp_read` 的
簇号→扇区换算，与此无关）。**实际值待 P&R 后回填记录表。**

**验证状态**：`begin/end`、`case/endcase`、`module/endmodule` 配对已核对无错配；
本机无仿真器，**尚未综合、尚未上板**。上板重点看第五节清单第 3 条。

---

## 五、上板回归清单（每次改完必跑）

- [ ] 4 张图自动轮播，转场无明显异常
- [ ] 按键切图响应正常，OSD 显示的"第几张"正确
- [ ] 切图耗时 OSD 的 T/W/L 三值有数且合理
- [ ] **F1 回归点**：T/W/L 描述的是**当前这张图**——数字在淡入结束那一刻跳变，不在停留期中间跳
- [ ] **F1 守底**：自动轮播 W=0.00、T≈含停留（1080P 约 4.1s）；KEY0 命中预读 W=0.00、T≈1.07
- [ ] 音频频谱柱状条随音调变化（8 个频点能分开）
- [ ] 亮度/对比度调节生效
- [ ] 数码管显示正常
- [ ] HDMI 稳定无闪屏、无花屏

---

## 六、记录表（每轮追加一行）

> 「各域 Fmax」列顺序固定为 **video / sd_card / ext_mem / clk(50M) / hdmi_5x**（单位 MHz），
> 取报告 `Path Group` 块里的 `Fmax` 行，不是 Clock Summary 的 `R-Freq`。

| 日期 | 版本/commit | 改了什么 | SWNS | 各域 Fmax | LUT | DSP | 上板 |
|---|---|---|---|---|---|---|---|
| 10-24 | `1158076` | 基线（封版） | −7.154 | 29.43 / 73.38 / 124.07 / 95.54 / 388.50 | 10914 | 22 | 通过 |
| 10-25 | `8389897`（E1 参数首次入库） | R0/E1：`place opt_timing ultra` | −6.699 | 28.04 / 72.80 / 126.73 / 85.50 / 337.50 | 10920 | 22 | 未跑 |
| 10-25 | 待提交 | R1：`img_scaler` 拆两级流水（`ysettle` 对齐）**← 已回滚，仅存档** | −6.699 | **30.20** / **73.83** / 122.90 / 88.20 / **385.20** | 10958 | 22 | 未跑（不再跑） |
| 10-25 | `8ca4952` | **E1 完整对平**：重跑 P&R，bit = `7c19647`（`ultra` 布局） | −6.699 | 28.04 / 72.80 / 126.73 / 85.50 / 337.50 | 10920 | 22 | **待上板** |
| 11-03 | 待提交 | **F1**：切图耗时发布时刻改挂"上屏 + 转场结束"（`switch_latency` 重写，纯观测） | 待跑 | 待跑 | 待跑 | 待跑 | 待上板 |
| | | | | | | | |
