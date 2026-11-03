// ============================================================================
// hdmi_phy_wrapper —— HDMI 物理层：4 条 TMDS 通道（3 数据 + 1 时钟）串行化输出
// ----------------------------------------------------------------------------
// 结构很简单，只有两件事：
//
// 一、位序翻转
//   加密的 HDMI 发送核给出的 10 位并行数据是 LSB 在前的，而 lane_lvds_10_1
//   里的串行化器按「送进去的最高位先出」工作。所以这里把 10 位整体倒序
//   （ch[0] 接到拼接结果的最高位），否则显示器收到的会是位序颠倒的乱码
//   —— 表现为有信号、有分辨率，但花屏或全绿。
//
// 二、四条通道各接一个串行化器
//   channel_0/1/2 是 TMDS 的三个数据通道（分别承载 B/G/R 及其控制位），
//   channel_clk 是 TMDS 时钟通道。四者用的是同一份 lane_lvds_10_1，
//   只有 I_data_in 不同，所以例化四份、参数一致即可。
//
// 注意：
//   · 文件名拼写是 hdmi_phy_warpper.v（少一个 p），但模块名是
//     hdmi_phy_wrapper —— 引用时以模块名为准，改名要连文件一起改；
//   · DEVICE 参数在一个 generate 里选具体器件的 ODDR 原语，本工程顶层传
//     "EG"（EG4S20），换芯片型号只需要改这一个参数；
//   · 只引出 _p 四根：pin.adc 里 IOSTANDARD 设成 LVDS33，同一差分对的 _N
//     由工具自动配对，代码里不用显式声明。
// ============================================================================

module hdmi_phy_wrapper#(
    parameter DEVICE = "EG"
    )(
    input wire      I_pixel_clk,
    input wire      I_serial_clk,
    input wire      I_rst,

    input wire[9:0] I_tmds_channel_0,
    input wire[9:0] I_tmds_channel_1,
    input wire[9:0] I_tmds_channel_2,
    input wire[9:0] I_tmds_channel_clk,
    
    output wire     O_tmds_ch0_p,
    output wire     O_tmds_ch1_p,
    output wire     O_tmds_ch2_p,
    output wire     O_tmds_clk_p
);
    
    wire[9:0] S_tmds_data_ch0;
    wire[9:0] S_tmds_data_ch1;
    wire[9:0] S_tmds_data_ch2;
    wire[9:0] S_tmds_data_clk;

    assign S_tmds_data_ch0 = {I_tmds_channel_0[0],
                              I_tmds_channel_0[1],
                              I_tmds_channel_0[2],
                              I_tmds_channel_0[3],
                              I_tmds_channel_0[4],
                              I_tmds_channel_0[5],
                              I_tmds_channel_0[6],
                              I_tmds_channel_0[7],
                              I_tmds_channel_0[8],
                              I_tmds_channel_0[9]};

    assign S_tmds_data_ch1 = {I_tmds_channel_1[0],
                              I_tmds_channel_1[1],
                              I_tmds_channel_1[2],
                              I_tmds_channel_1[3],
                              I_tmds_channel_1[4],
                              I_tmds_channel_1[5],
                              I_tmds_channel_1[6],
                              I_tmds_channel_1[7],
                              I_tmds_channel_1[8],
                              I_tmds_channel_1[9]};

    assign S_tmds_data_ch2 = {I_tmds_channel_2[0],
                              I_tmds_channel_2[1],
                              I_tmds_channel_2[2],
                              I_tmds_channel_2[3],
                              I_tmds_channel_2[4],
                              I_tmds_channel_2[5],
                              I_tmds_channel_2[6],
                              I_tmds_channel_2[7],
                              I_tmds_channel_2[8],
                              I_tmds_channel_2[9]};

    assign S_tmds_data_clk = {I_tmds_channel_clk[0],
                              I_tmds_channel_clk[1],
                              I_tmds_channel_clk[2],
                              I_tmds_channel_clk[3],
                              I_tmds_channel_clk[4],
                              I_tmds_channel_clk[5],
                              I_tmds_channel_clk[6],
                              I_tmds_channel_clk[7],
                              I_tmds_channel_clk[8],
                              I_tmds_channel_clk[9]};


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u0_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),

        .I_data_in    ( S_tmds_data_ch0 ),
        .O_serial_out ( O_tmds_ch0_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u1_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),
        
        .I_data_in    ( S_tmds_data_ch1 ),
        .O_serial_out ( O_tmds_ch1_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u2_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),
        
        .I_data_in    ( S_tmds_data_ch2 ),
        .O_serial_out ( O_tmds_ch2_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u3_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst           ),
        
        .I_data_in    ( S_tmds_data_clk ),
        .O_serial_out ( O_tmds_clk_p    )
    );


endmodule





