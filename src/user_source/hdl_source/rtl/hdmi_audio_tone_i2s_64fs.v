
// ============================================================================
// hdmi_audio_tone_i2s_64fs —— I2S 测试音发生器（8 音循环，64fs）
// ----------------------------------------------------------------------------
// 由 12.288MHz MCLK 直接产生 I2S 的 BCLK / LRCK / DOUT：
//   · BCLK：每个 MCLK 翻转一次 → BCLK = MCLK/2 = 6.144MHz；
//   · 每个声道 64 bit（这就是名字里 "64fs" 的含义），一帧 128 bit；
//   · LRCK = BCLK/128 = 48kHz，正好是 I2S 的标准采样率。
//   24bit 样本放在每声道的最高 24 位（bit[63:40]），其余补 0 —— 与
//   I2S_receiver 里「取 [63:40]」的接收约定一一对应，两边改一处必须改另一处。
//
// 8 个音（C4~C5）循环，每个音保持 NOTE_HOLD_FRAMES 个采样（默认 24000 = 0.5s）。
// 波形为方波：相位累加器最高位决定输出 +AMP 还是 -AMP（24bit 补码），
// 占空比 50%。目的不是好听，而是给出一个频率精确、便于频谱模块计数的激励。
// 左右声道用同一个样本。
// 用途：没有真实音频源时给 HDMI 音频链路一个可听、可测的激励，
//       同时让 audio_spectrum 的柱高随音调阶梯变化。
// ============================================================================
module hdmi_audio_tone_i2s_64fs #(
    parameter [31:0] PHASE_INC = 32'd39370534,   // 兼容旧顶层，实际本版内部不用它
    parameter [23:0] AMP = 24'd2000000,          // 幅度（无符号正值，负半周取补码）
    parameter integer NOTE_HOLD_FRAMES = 24000   // 每个音持续 0.5s @ 48kHz
)(
    input  wire I_mclk,      // 12.288MHz
    input  wire I_rst,
    output reg  O_i2s_BCLK,
    output reg  O_i2s_LRCK,
    output reg  O_i2s_DOUT
);

// do/re/mi/fa/so/la/si/do  （C4 D4 E4 F4 G4 A4 B4 C5）
// 相位增量按 48kHz 采样率计算
function [31:0] note_inc_lut;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: note_inc_lut = 32'd23409862; // do  C4 261.63Hz
            3'd1: note_inc_lut = 32'd26276681; // re  D4 293.66Hz
            3'd2: note_inc_lut = 32'd29494578; // mi  E4 329.63Hz
            3'd3: note_inc_lut = 32'd31248410; // fa  F4 349.23Hz
            3'd4: note_inc_lut = 32'd35075155; // so  G4 392.00Hz
            3'd5: note_inc_lut = 32'd39370534; // la  A4 440.00Hz
            3'd6: note_inc_lut = 32'd44191930; // si  B4 493.88Hz
            default: note_inc_lut = 32'd46819716; // do  C5 523.25Hz
        endcase
    end
endfunction

reg  [5:0]  S_bit_cnt;
reg  [63:0] S_shift_reg;
reg  [31:0] S_phase_acc;
reg  [23:0] S_sample_word;
reg  [23:0] S_sample_next;
reg  [2:0]  S_note_idx;
reg [15:0]  S_note_frame_cnt;

wire [31:0] W_note_inc = note_inc_lut(S_note_idx);

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK      <= 1'b0;
        O_i2s_LRCK      <= 1'b0;
        O_i2s_DOUT      <= 1'b0;
        S_bit_cnt       <= 6'd0;
        S_shift_reg     <= 64'd0;
        S_phase_acc     <= 32'd0;
        S_sample_word   <= 24'd0;
        S_sample_next   <= AMP;
        S_note_idx      <= 3'd0;
        S_note_frame_cnt<= 16'd0;
    end
    else begin
        // 保持和你现有接收端兼容的 64fs 发送时序
        O_i2s_BCLK <= ~O_i2s_BCLK;

        if (O_i2s_BCLK == 1'b1) begin
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin
                S_bit_cnt <= 6'd0;

                if (O_i2s_LRCK == 1'b0) begin
                    // 左声道发完，右声道复用同一个样本
                    O_i2s_LRCK  <= 1'b1;
                    S_shift_reg <= {S_sample_word[23:0], 40'd0};
                end
                else begin
                    // 右声道发完，进入下一帧样本
                    O_i2s_LRCK  <= 1'b0;

                    S_phase_acc <= S_phase_acc + W_note_inc;
                    if (S_phase_acc[31])
                        S_sample_next <= AMP;
                    else
                        S_sample_next <= (~AMP) + 24'd1;   // -AMP 的 24bit 补码

                    S_sample_word <= S_sample_next;
                    S_shift_reg   <= {S_sample_next[23:0], 40'd0};

                    if (S_note_frame_cnt == NOTE_HOLD_FRAMES - 1) begin
                        S_note_frame_cnt <= 16'd0;
                        if (S_note_idx == 3'd7)
                            S_note_idx <= 3'd0;
                        else
                            S_note_idx <= S_note_idx + 3'd1;
                    end
                    else begin
                        S_note_frame_cnt <= S_note_frame_cnt + 16'd1;
                    end
                end
            end
            else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
