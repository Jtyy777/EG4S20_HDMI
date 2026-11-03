


// ============================================================================
// global_def.v —— SDRAM 控制器全局位宽与刷新参数
// ----------------------------------------------------------------------------
// 32bit 数据 / 21bit 字地址 / 4bit DM / 11bit 行地址 / 2bit bank
// SELF_REFRESH_INTERVAL = 64ms ÷ 行数 ÷ 2（按 125MHz 折算），单位是时钟周期；
// 改动 SDRAM 型号或 ext_mem_clk 频率时必须同步改这里。
// ============================================================================
`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

`define	  SDR_CLK_PERIOD				1000000000/125000000
`define   SELF_REFRESH_INTERVAL			64000000/`SDR_CLK_PERIOD/2**(`ROW_WIDTH)

