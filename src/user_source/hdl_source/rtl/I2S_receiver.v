

// ============================================================================
// I2S_receiver —— I2S 接收：串行 PCM → 24bit 左右声道并行
// ----------------------------------------------------------------------------
// 输入 BCLK / LRCK / DOUT 先各打 3 拍同步（异步输入）。
// 在 BCLK 上升沿把 DOUT 移入 64bit 移位寄存器，用 LRCK 的采样值选声道：
//   LRCK = 1 → 右声道移位，LRCK = 0 → 左声道移位
// LRCK 上升沿锁存左声道，下降沿那一拍输出：两声道各取高 24bit（[63:40]）
// 送到 O_audio_left_data / O_audio_right_data。
// O_audio_valid 每个立体声采样周期一拍，供 audio_arc_calculate 与
// audio_spectrum 使用（48kHz）。
// 注意：本例发送端是 64fs 的 I2S，取高 24bit 即为有效数据。
// ============================================================================
module I2S_receiver (
    input wire       I_clk,
    input wire       I_rst,
  
    input wire       I_i2s_BCLK,
    input wire       I_i2s_LRCK,
    input wire       I_i2s_DOUT,

    output reg       O_audio_valid,
    output reg[23:0] O_audio_left_data,
    output reg[23:0] O_audio_right_data
);


    reg       S_i2s_bclk_1d;
    reg       S_i2s_bclk_2d;
    reg       S_i2s_bclk_3d;
  
    wire      S_i2s_bclk_p_edge;  
  
    reg       S_i2s_lrck_1d;
    reg       S_i2s_lrck_2d;
    reg       S_i2s_lrck_3d;
    reg       S_i2s_lrck_sync;
    wire      S_i2s_lrck_p_edge;
    wire      S_i2s_lrck_n_edge;
  
    reg       S_i2s_dout_1d;
    reg       S_i2s_dout_2d;
    reg       S_i2s_dout_3d;

    reg[63:0] S_left_shift_data;
    reg[63:0] S_right_shift_data;

    reg       S_left_data_valid;
    reg       S_right_data_valid;

    reg[63:0] S_left_data_lock;

    always @(posedge I_clk) begin
        S_i2s_bclk_1d <= I_i2s_BCLK;
        S_i2s_bclk_2d <= S_i2s_bclk_1d;
        S_i2s_bclk_3d <= S_i2s_bclk_2d;

        S_i2s_lrck_1d <= I_i2s_LRCK;
        S_i2s_lrck_2d <= S_i2s_lrck_1d;
        S_i2s_lrck_3d <= S_i2s_lrck_2d;

        S_i2s_dout_1d <= I_i2s_DOUT;
        S_i2s_dout_2d <= S_i2s_dout_1d;
        S_i2s_dout_3d <= S_i2s_dout_2d;
    end

    assign S_i2s_bclk_p_edge = ~S_i2s_bclk_3d & S_i2s_bclk_2d;


    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge)
            S_i2s_lrck_sync <= S_i2s_lrck_3d;
        else
            S_i2s_lrck_sync <= S_i2s_lrck_sync;
    end

    assign S_i2s_lrck_p_edge = ~S_i2s_lrck_sync & S_i2s_lrck_3d;

    assign S_i2s_lrck_n_edge = ~S_i2s_lrck_3d & S_i2s_lrck_sync;

    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge)
            if(S_i2s_lrck_sync)
                begin
                    S_right_shift_data <= {S_right_shift_data[62:0],S_i2s_dout_3d};
                    S_left_shift_data  <= 'd0;
                end
            else
                begin
                    S_right_shift_data <= 'd0;
                    S_left_shift_data  <= {S_left_shift_data[62:0],S_i2s_dout_3d};
                end
        else
            begin
                S_right_shift_data <= S_right_shift_data;
                S_left_shift_data  <= S_left_shift_data;
            end
    end


    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge && S_i2s_lrck_p_edge)
            S_left_data_valid <= 1'b1;
        else
            S_left_data_valid <= 1'b0;
    end

    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge && S_i2s_lrck_n_edge)
            S_right_data_valid <= 1'b1;
        else
            S_right_data_valid <= 1'b0;
    end

    always @(posedge I_clk) begin
        if(S_left_data_valid)
            S_left_data_lock <= S_left_shift_data;
        else
            S_left_data_lock <= S_left_data_lock;
    end

    always @(posedge I_clk) begin
        if(S_right_data_valid)
            begin
                O_audio_valid      <= 1'b1;
                O_audio_left_data  <= S_left_data_lock[63:40];
                O_audio_right_data <= S_right_shift_data[63:40];
            end
        else
            begin
                O_audio_valid      <= 1'b0;
                O_audio_left_data  <= 'd0;
                O_audio_right_data <= 'd0;
            end
    end

    // always @(posedge I_clk) begin
    //     if(S_i2s_bclk_p_edge)
    //         begin
    //             if(S_i2s_lrck_3d)
    //                 begin
    //                     S_right_channel_cnt <= S_right_channel_cnt + 1'b1;
    //                     S_left_channel_cnt  <= 'd0;
    //                 end
    //             else    
    //                 begin
    //                     S_right_channel_cnt <= 'd0;
    //                     S_left_channel_cnt  <= S_left_channel_cnt + 1'b1;
    //                 end
    //         end
    //     else
    //         begin
    //             S_right_channel_cnt <= S_right_channel_cnt;
    //             S_left_channel_cnt  <= S_left_channel_cnt;
    //         end
    // end


    

endmodule