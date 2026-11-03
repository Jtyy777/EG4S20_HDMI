// ============================================================================
// lane_lvds_10_1 —— 10 位并行 → DDR 串行的一路 TMDS lane（单通道串行化器）
// ----------------------------------------------------------------------------
// HDMI 的每条 TMDS 通道每像素要发 10 bit。用 ODDR 在时钟的上下沿各出 1 bit，
// 一个 serial_clk 出 2 bit，所以 10 bit 需要 5 个 serial_clk：
//     serial_clk = 5 × pixel_clk
// 本工程 pixel_clk = 25.175MHz（640×480@60），serial_clk = 125.875MHz。
//
// 一、装载节奏怎么来
//   S_odd_even 在 pixel_clk 域每个像素翻转一次。它被两级打到 serial_clk 域后
//   做一次异或（S_load_en = 1d ^ 2d），正好在每次变化时给出一个单拍脉冲
//   —— 这个脉冲就是「该装载新像素了」。这个脉冲再打两拍（S_load_en_2d）
//   才真正用于装载，作用是让装载沿与后面的移位流水对齐、给使能路径留余量。
//
// 二、串并转换过程
//   装载：S_shift_data <= I_data_in（10 位一次性进）
//   其余每拍：左移 2 位，同时把当前 [9:8] 交给 ODDR
//   于是发出去的顺序是 bit9、bit8 → bit7、bit6 → … → bit1、bit0，
//   也就是 MSB first。这也是上一层要把 10 位倒序的原因（见 hdmi_phy_wrapper）。
//
// 三、器件适配
//   generate 里按 DEVICE 选不同型号的 ODDR 原语（EF2/EF3/EF4/SF1/EG/PH1A/
//   PH1P/DR1），端口完全一致，本工程走 "EG" 分支。换芯片只改 DEVICE 即可，
//   但这套原语名必须和所用器件库对得上，否则综合期就会报找不到模块。
//
// 四、注意
//   · S_odd_even_1d / _2d 只在 serial_clk 上打拍、没有复位，上电初值不确定。
//     这不影响功能：它们只用于产生装载脉冲，下一个像素边沿就会重新同步；
//   · S_oddr_data 也是无复位的寄存器，理由同上；
//   · ODDR 的 d0/d1 谁对应上升沿、谁对应下降沿，由器件原语定义。若把位序
//     接反，现象是颜色错乱而不是完全没有画面 —— 这层调试靠换 d0/d1 试。
// ============================================================================


module lane_lvds_10_1 #(
    parameter DEVICE = "EG"
    )(
    input wire      I_pixel_clk,
    input wire      I_serial_clk,
    input wire      I_rst,

    input wire[9:0] I_data_in,

    output wire     O_serial_out
);

    reg      S_odd_even;     
    reg      S_odd_even_1d;  
    reg      S_odd_even_2d;  
    reg      S_load_en;      
    reg      S_load_en_1d;   
    reg      S_load_en_2d;   
    reg[9:0] S_shift_data;   
    reg[1:0] S_oddr_data;    


    always @(posedge I_pixel_clk or posedge I_rst) begin
        if(I_rst)
            S_odd_even <= 1'b0;
        else
            S_odd_even <= ~S_odd_even;
    end

    always @(posedge I_serial_clk) begin
        S_odd_even_1d <= S_odd_even;
        S_odd_even_2d <= S_odd_even_1d;
    end

    always @(posedge I_serial_clk) begin
        S_load_en    <= S_odd_even_1d ^ S_odd_even_2d;
        S_load_en_1d <= S_load_en;
        S_load_en_2d <= S_load_en_1d;
    end

    always @(posedge I_serial_clk or posedge I_rst) begin
        if(I_rst)
            S_shift_data <= 'd0;
        else
            if(S_load_en_2d)
                S_shift_data <= I_data_in;
            else 
                S_shift_data <= S_shift_data << 2;
    end

    always @(posedge I_serial_clk) begin
        S_oddr_data <= S_shift_data[9:8];
    end

    generate
        if(DEVICE == "EF2")
            begin
                EF2_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF3")
            begin
                EF3_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF4")
            begin
                EF4_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "SF1")
            begin
                SF1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EG")
            begin
                EG_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1A")
            begin
                PH1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1P")
            begin
                PH1P_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "DR1")
            begin
                DR1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
    endgenerate
    
endmodule