// ============================================================================
// pixel_coord_gen.v  —— 像素坐标生成器（纯自研模块）
//
// 功能：根据 video_delay 输出的 de/vs（已经过 21 拍延迟对齐）重建当前
//       有效像素的 (x, y) 坐标，供转场特效、OSD、频谱叠加等后处理模块使用。
//
// 原理：de 高电平期间逐像素计数 x（0~639），行末翻 y，vs 上升沿把 (x,y) 归零。
//       复位动作放在 vs 上升沿而不是 de 的下降沿，是因为 vs 在消隐期就已
//       出现，能保证「第一个有效像素到来时坐标已经是 (0,0)」。
//       推进一步发生在 de 当拍的末尾，所以输出 y/x 在 de 期间始终指向
//       「正在输出的这个像素」，下游可以当拍直接用。
//       消隐期 de=0，计数原地不动 —— 不会把消隐的几十个像素算进坐标。
// ============================================================================

module pixel_coord_gen #(
    parameter integer H_ACTIVE = 640,
    parameter integer V_ACTIVE = 480
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        de,
    input  wire        vs,

    output reg  [10:0] x,     // 当前像素列 0~H_ACTIVE-1
    output reg  [10:0] y      // 当前像素行 0~V_ACTIVE-1
);

    reg vs_d1;
    wire vs_rise;

    assign vs_rise = vs & ~vs_d1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vs_d1 <= 1'b0;
            x     <= 11'd0;
            y     <= 11'd0;
        end else begin
            vs_d1 <= vs;

            if (vs_rise) begin
                y <= 11'd0;
                x <= 11'd0;
            end else if (de) begin
                if (x >= H_ACTIVE - 1) begin
                    x <= 11'd0;
                    y <= (y >= V_ACTIVE - 1) ? 11'd0 : y + 11'd1;
                end else begin
                    x <= x + 11'd1;
                end
            end
        end
    end

endmodule
