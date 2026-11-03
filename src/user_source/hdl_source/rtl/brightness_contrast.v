// ============================================================================
// brightness_contrast —— 亮度 / 对比度实时调节
// ----------------------------------------------------------------------------
// 功能：对 24bit RGB 像素做亮度与对比度调节
//   亮度：+bright 偏移（-96 ~ +96，8bit 二进制补码）
//   对比度：×contrast 缩放（0.50x ~ 2.00x，8.8 定点，256 = 1.0x）
//
// 公式（定点 8.8）
//   out = clamp( (in - 128) * contrast_q8 / 256 + 128 + bright )
//
// 位宽说明（乘法部分）
//   contrast_q8 已扩成 10bit（128~512），(in-128) 为 ±128，
//   乘积绝对值最大 128*512 = 65536，必须用 32bit 中间量才能保住符号。
//   旧代码用 16bit：centered(16bit) * cst(9bit) 后取 prod[15:0]，
//   当 |centered*cst| > 32767（即像素偏离 128 超过约 68 且对比度较高）时
//   16bit 补码溢出，结果符号翻转，亮部会被算成暗部（例如 in=228、
//   对比度 1.88x 时本应是 255，旧代码算成 59）。
//   现在统一扩到 32bit 补码运算，全程不溢出。
// ============================================================================

module brightness_contrast (
    input  wire        clk,
    input  wire        rst,
    input  wire        de,

    input  wire [23:0] rgb_in,
    input  wire [7:0]  bright,        // 有符号亮度偏移，-96~+96（二进制补码）
    input  wire [9:0]  contrast_q8,   // 对比度定点，256=1.0x，512=2.0x

    output wire [23:0] rgb_out,
    output reg         de_out          // 延迟后的 de（1 拍延迟，与 rgb_out 对齐）
);

    // ---- 单通道调节（纯组合，无符号补码展开）----
    function [7:0] adj_channel;
        input [7:0] in_val;
        input [7:0] brt;
        input [9:0] cst_q8;
        reg [31:0] centered;      // (in-128) 的 32bit 补码
        reg [31:0] prod;          // centered * cst_q8 的 32bit 补码
        reg [31:0] scaled;        // prod / 256（符号保持）
        reg [31:0] added;         // scaled + bright + 128
        begin
            // centered = (in - 128)
            // 注意：in_val 是 0~255 的**无符号**像素值，必须零扩展；
            // 旧代码写成 {in_val[7], ..., in_val}（符号扩展），
            // 导致 in>=128 时被当成负数（228 变成 -28），亮部整体算错。
            centered = { 24'd0, in_val } - 32'd128;

            // prod = centered * cst_q8
            // |结果| <= 128*512 = 65536，32bit 补码内绝不溢出，
            // 无符号位模式乘法与有符号乘法低 32 位结果一致。
            prod = centered * { 22'd0, cst_q8 };

            // scaled = prod / 256 = 算术右移 8 位（高 8 位是符号扩展）
            scaled = { {8{prod[31]}}, prod[31:8] };

            // added = scaled + bright(符号扩展) + 128
            added = scaled
                    + { {24{brt[7]}}, brt }
                    + 32'd128;

            // clamp 到 0~255
            if (added[31])
                adj_channel = 8'd0;           // 负数 -> 0
            else if (added > 32'd255)
                adj_channel = 8'd255;
            else
                adj_channel = added[7:0];
        end
    endfunction

    reg [7:0] r_adj, g_adj, b_adj;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_adj  <= 8'd0;
            g_adj  <= 8'd0;
            b_adj  <= 8'd0;
            de_out <= 1'b0;
        end else begin
            de_out <= de;
            if (de) begin
                r_adj <= adj_channel(rgb_in[23:16], bright, contrast_q8);
                g_adj <= adj_channel(rgb_in[15:8],  bright, contrast_q8);
                b_adj <= adj_channel(rgb_in[7:0],   bright, contrast_q8);
            end
        end
    end

    assign rgb_out = { r_adj, g_adj, b_adj };

endmodule
