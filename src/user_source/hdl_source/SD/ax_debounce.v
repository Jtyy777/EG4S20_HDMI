// ============================================================================
// ax_debounce —— 通用按键消抖（电平稳定 20ms 后才认）
// ----------------------------------------------------------------------------
// 用途：把机械按键的抖动滤掉，并输出消抖后的电平、上升沿、下降沿。
//
// 工作原理：
//   · DFF1/DFF2 两级同步，把异步按键打进来；DFF1 ^ DFF2 表示「输入还在变」，
//     一变化就把计时器清零重来（q_reset）。
//   · q_reg 从 0 数到 TIMER_MAX_VAL = 20ms × 50MHz = 1,000,000 才停，
//     计时期间输入一动就归零，所以必须连续稳定 20ms 才认账。
//   · 数满后 button_out 才跟随 DFF2，再用一拍延迟做边沿检测。
//
// 注意：
//   · 复位值 button_out = 1'b1 —— 按键板上拉、按下为低电平，所以按「复位后
//     视为未按下」。对应关系：button_negedge 是「按下」，button_posedge 是
//     「松开」，取边沿时要按这个极性选。
//   · 本模块当前没有被工程里任何地方实例化（sd_card_bmp.v 用的是自带
//     key_press_debounce），保留作通用参考实现。
//   · 参数是 parameter，改时钟频率时记得同步改 FREQ，否则消抖时间会跟着偏。
// ============================================================================
module  ax_debounce 
(
    input       clk, 
    input       rst, 
    input       button_in,
    output reg  button_posedge,
    output reg  button_negedge,
    output reg  button_out
);
//// ---------------- internal constants --------------
parameter N = 32 ;           // debounce timer bitwidth
parameter FREQ = 50;         //model clock :Mhz
parameter MAX_TIME = 20;     //ms
localparam TIMER_MAX_VAL =   MAX_TIME * 1000 * FREQ;
////---------------- internal variables ---------------
reg  [N-1 : 0]  q_reg;      // timing regs
reg  [N-1 : 0]  q_next;
reg DFF1, DFF2;             // input flip-flops
wire q_add;                 // control flags
wire q_reset;
reg button_out_d0;
//// ------------------------------------------------------

////contenious assignment for counter control
assign q_reset = (DFF1  ^ DFF2);          // xor input flip flops to look for level chage to reset counter
assign q_add = ~(q_reg == TIMER_MAX_VAL); // add to counter when q_reg msb is equal to 0
    
//// combo counter to manage q_next 
always @ ( q_reset, q_add, q_reg)
begin
    case( {q_reset , q_add})
        2'b00 :
                q_next <= q_reg;
        2'b01 :
                q_next <= q_reg + 1;
        default :
                q_next <= { N {1'b0} };
    endcase     
end

//// Flip flop inputs and q_reg update
always @ ( posedge clk or posedge rst)
begin
    if(rst == 1'b1)
    begin
        DFF1 <= 1'b0;
        DFF2 <= 1'b0;
        q_reg <= { N {1'b0} };
    end
    else
    begin
        DFF1 <= button_in;
        DFF2 <= DFF1;
        q_reg <= q_next;
    end
end

//// counter control
always @ ( posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		button_out <= 1'b1;
    else if(q_reg == TIMER_MAX_VAL)
        button_out <= DFF2;
    else
        button_out <= button_out;
end

always @ ( posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		button_out_d0 <= 1'b1;
		button_posedge <= 1'b0;
		button_negedge <= 1'b0;
	end
	else
	begin
		button_out_d0 <= button_out;
		button_posedge <= ~button_out_d0 & button_out;
		button_negedge <= button_out_d0 & ~button_out;
	end	
end
endmodule


