// ============================================================================
// bmp_read —— BMP 文件扫描与加载（FAT32 目录解析）
// ----------------------------------------------------------------------------
// 功能说明
//   上电扫一次根目录，定位前 scan_target_count 个 .BMP 的起始扇区；
//   之后按扇区号加载指定文件，把像素按文件行序送给 img_scaler。
//
// 不扫扇区的原因
//   早期实现从扇区 0 逐扇区搜 "BM" 魔数，60GB FAT32 卡要扫 3 万多个无用扇区
//   （引导区 + FAT 表），200kHz SPI 下超过 10 分钟，表现为状态码停在 2
//   （ST_SCAN）黑屏。改为解析 FAT32 目录表后秒级出图，换卡也兼容。
//
// 扫描流程
//   1. 读扇区 0，判定是 MBR 还是整卡 BPB，得到分区基址
//   2. 读分区内 BPB，取 reserved / num_fats / fat_size / spc / root_cluster
//   3. 数据区起始扇区 = reserved + num_fats × fat_size
//   4. 逐 32 字节目录项解析根目录，取前 N 个扩展名为 .BMP 文件的起始簇
//   5. 起始扇区 = 分区基址 + 数据区起始 + (簇号 - 2) × spc
//
// 加载流程
//   ST_LOAD_HDR 读文件头校验 → ST_LOAD_WAIT 等写 FIFO 应答 →
//   ST_LOAD_DATA 读像素（每读完一个簇查 FAT 取下一簇，支持碎片化文件）
//
// 支持的 BMP
//   未压缩 24/32bpp（32bpp 丢弃 alpha）、宽高 1..4096、自底向上；
//   行尾 4 字节对齐填充被剔除，像素按文件行序输出。
//
// 对外接口与原模块完全兼容。
// ============================================================================

module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：解析 FAT32 目录，定位前 scan_target_count 张 BMP 的起始扇区
    input                       scan_start,
    input  [31:0]               scan_start_sector,   // 兼容原版的保留
    input  [31:0]               scan_max_sector,     // 兼容原版的保留
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    // 0=SD未初始化 1=IDLE 2=扫描/读头 3=等写SDRAM 4=读像素 5=查FAT簇链
    output reg [3:0]            state_code,

    // 本次加载的源图尺寸（BMP 头解析结果，供 img_scaler 用）
    output reg [15:0]           img_w,
    output reg [15:0]           img_h,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    // 源像素流：按 BMP 文件行序逐个像素输出，已剔除行尾 4 字节对齐填充
    output reg                  pix_de,
    output reg [23:0]           pix_data,     // {R,G,B}
    output reg [15:0]           pix_x         // 0 .. img_w-1
);

// ---------------------------------------------------------------------------
// 状态定义
// ---------------------------------------------------------------------------
localparam ST_IDLE        = 4'd0;
localparam ST_SCAN_MBR    = 4'd1;   // 读物理扇区0（MBR，提取分区起始扇区）
localparam ST_SCAN_BPB    = 4'd2;   // 读分区内 BPB 扇区
localparam ST_SCAN_ROOT   = 4'd3;   // 读根目录扇区
localparam ST_SCAN_NEXT   = 4'd4;   // 读下一个根目录扇区
localparam ST_LOAD_HDR    = 4'd5;   // 读 BMP 头校验
localparam ST_LOAD_WAIT   = 4'd6;   // 等待写 SDRAM 应答
localparam ST_LOAD_DATA   = 4'd7;   // 读像素数据
localparam ST_LOAD_FAT    = 4'd8;   // 读完一个簇 → 查 FAT 找下一簇（碎片化支持）

reg [3:0]  state;

// ---------------------------------------------------------------------------
// BPB / FAT 布局寄存器
// ---------------------------------------------------------------------------
reg [31:0] reserved_sectors;      // BPB 偏移 14，2 字节
reg [31:0] num_fats;              // BPB 偏移 16，1 字节
reg [31:0] fat_size;              // BPB 偏移 36，4 字节
reg [31:0] sectors_per_cluster;   // BPB 偏移 13，1 字节
reg [31:0] root_cluster;          // BPB 偏移 44，4 字节
reg [31:0] first_data_sector;     // = reserved + num_fats * fat_size（分区内相对扇区）
reg [31:0] root_dir_sector;       // = first_data + (root_cluster-2) * spc（物理扇区）
reg [31:0] root_dir_scan_cnt;     // 已扫根目录扇区计数
reg [31:0] root_dir_scan_max;     // 根目录最多扫多少扇区（防死循环）

// 目录项解析
reg         dir_lfn;              // 当前目录项是否为长文件名项（跳过）
reg         dir_del;              // 当前目录项已被删除（首字节 0xE5），必须跳过
reg         dir_attr_dir;         // 是否目录
reg [7:0]   dir_name8;            // 短文件名前 8 字节（用于判断 .BMP 扩展名）
reg [7:0]   dir_ext0, dir_ext1, dir_ext2;  // 扩展名 B M P
reg [31:0]  dir_start_cluster;    // 起始簇号

// 扫描结果
reg [2:0]   found_cnt;

// ---------------------------------------------------------------------------
// 加载相关寄存器（保留原逻辑）
// ---------------------------------------------------------------------------
reg [9:0]   rd_cnt;
reg [7:0]   header_0;
reg [7:0]   header_1;
reg [31:0]  file_len;
reg [31:0]  pixel_offset;
reg [31:0]  width;
reg [31:0]  height;
reg [15:0]  bit_count;
reg [31:0]  compression;

reg [31:0]  load_sector_latched;
reg [31:0]  bmp_len_cnt;

// ---------------------------------------------------------------------------
// 簇链（支持碎片化文件）
//
// 早期实现读像素时把扇区号一路 +1，隐含"文件在卡上物理连续"的假设。SD 卡被
// wipe / 重拷后极易碎片化（实测某图簇链为 3,4,5,6,122,123,…,181，前 4 簇后
// 直接跳到 122），此时 +1 读到的是别人的数据，表现为"前半张是这张图、后半张
// 变成别的图"（用户说的"二合一"）。
// 现在每读完一个簇就查 FAT 取下一簇，按 (簇号-2)×spc 重算物理扇区。
// ---------------------------------------------------------------------------
reg [31:0]  cur_clus;         // 当前正在读的簇号
reg [31:0]  sec_in_clus;      // 本簇内已读扇区数（0..spc-1）
reg [31:0]  fat_item;         // 从 FAT 读回的 32 位表项（低 28 位有效）
reg [31:0]  fat_off;          // 目标表项在当前 FAT 扇区内的字节偏移
reg [9:0]   fat_cnt;          // FAT 扇区内字节计数
reg [31:0]  fat_lba;          // FAT1 表起始物理扇区
reg [4:0]   spc_shift;        // log2(sectors_per_cluster)

// FAT32 规定 BPB_SecPerClus 只能是 2 的幂（1/2/4/.../128），
// 这里用移位代替除法 —— 安路 TD 对组合除法器不可靠（踩过）。
always @* begin
    case (sectors_per_cluster)
        32'd1   : spc_shift = 5'd0;
        32'd2   : spc_shift = 5'd1;
        32'd4   : spc_shift = 5'd2;
        32'd8   : spc_shift = 5'd3;
        32'd16  : spc_shift = 5'd4;
        32'd32  : spc_shift = 5'd5;
        32'd64  : spc_shift = 5'd6;
        32'd128 : spc_shift = 5'd7;
        default : spc_shift = 5'd6;      // 兜底按 32KB/簇
    endcase
end

// FAT 表项的「本拍最新值」：sd_sec_read_end 与最后一个 data_valid 可能是同一拍，
// 而非阻塞赋值下 fat_item 要下一拍才齐；跳转判断若直接用 fat_item，
// 最高字节永远慢一拍 → 算出的簇号高 8 位是上一次的残值。
wire [31:0] fat_item_n = {
    (fat_cnt == (fat_off + 32'd3)) ? sd_sec_read_data : fat_item[31:24],
    (fat_cnt == (fat_off + 32'd2)) ? sd_sec_read_data : fat_item[23:16],
    (fat_cnt == (fat_off + 32'd1)) ? sd_sec_read_data : fat_item[15:8],
    (fat_cnt ==  fat_off)          ? sd_sec_read_data : fat_item[7:0]
};

// ---------------------------------------------------------------------------
// 像素流几何（支持任意分辨率）
//   行字节数（含 4 字节对齐填充）：stride = (w × bpp/8 + 3) & ~3
//   输出只取每行前 w × bpp/8 字节，填充字节丢弃；
//   字节相位 byte_ph 相对像素数据起点 pixel_offset 计数，
//   所以 pixel_offset 不是 3 的倍数也不会错位。
// ---------------------------------------------------------------------------
reg [31:0]  row_byte;        // 行内字节偏移 0..stride-1
reg [1:0]   byte_ph;         // 像素内字节序号
reg [23:0]  px_acc;          // 拼装中的像素
reg [15:0]  px_cnt;          // 本行已输出像素数（= 下一个像素的 x）
reg [15:0]  py_cnt;          // 已输出的行数
reg         img_32bpp;       // 1 = 32 位 BMP（第 4 字节 alpha 丢弃）

//将img_w的运算综合成移位加法，省去1DSP
wire [31:0] hdr_row_bytes = img_32bpp ? {14'd0, img_w, 2'b00}
                                      : ({16'd0, img_w} * 32'd3);
wire [31:0] hdr_row_stride = (hdr_row_bytes + 32'd3) & ~32'd3;

// BMP 头合法性：只接受未压缩 24/32 位；尺寸限制在 1..4096
wire [15:0] hdr_w = width[15:0];
wire [15:0] hdr_h = height[31] ? (~height[15:0] + 16'd1) : height[15:0];
wire hdr_ok = (header_0 == "B") &&
              (header_1 == "M") &&
              ((bit_count == 16'd24) || (bit_count == 16'd32)) &&
              (compression == 32'd0) &&
              (hdr_w != 16'd0) && (hdr_h != 16'd0) &&
              (hdr_w <= 16'd4096) && (hdr_h <= 16'd4096);

wire bmp_data_valid;

assign ready = (state == ST_IDLE);
// 必须限定 ST_LOAD_DATA：查 FAT 簇链时同样会拉高 sd_sec_read_data_valid，
//   而那时 bmp_len_cnt 也落在 [pixel_offset, file_len) 内。不加状态条件，
//   FAT 表那 512 字节会被当成像素吐出去（4 字节簇号变 1 个花点，整行错位）。
assign bmp_data_valid = (state == ST_LOAD_DATA) &&
                        (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);

// ---------------------------------------------------------------------------
// rd_cnt：扫描/读头阶段的数据字节计数
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN_MBR) || (state == ST_SCAN_BPB) || (state == ST_SCAN_ROOT) || (state == ST_SCAN_NEXT) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

// ---------------------------------------------------------------------------
// 扇区 0 类型判定：区分「整卡 BPB（无分区表）」与「MBR（有分区表）」。
//
// 不能只看偏移 510/511 的 0x55AA：整卡 FAT32 的 BPB 在 510/511 也恰好是
// 0x55AA（total_sectors 字段巧合）；也不能只看分区类型字节（可能非常见类型
// 而被漏判）。改用 BPB 特征反向判定：
//   · 偏移 0 = 0xEB / 0xE9（x86 短跳转）且偏移 82~86 = "FAT32"
//     → 整卡 FAT32，无分区表，part_start = 0
//   · 否则若 510/511 = 0x55AA → 是 MBR，从第一分区表项（偏移 454~457）
//     取分区起始扇区，再进分区读 BPB
// ---------------------------------------------------------------------------
reg         bpb_is_bpb;          // 扇区 0 是否具备 BPB 特征（跳转指令 + "FAT32"）
reg [7:0]   boot_jmp;            // 偏移 0（跳转指令）
reg [7:0]   fs_type_0, fs_type_1, fs_type_2, fs_type_3, fs_type_4;  // 偏移 82~86
reg         mbr_sig_valid;       // 0x55AA 签名
reg [7:0]   mbr_sig_0;           // 偏移 510
reg [31:0]  mbr_raw_start;       // 从分区表项读到的原始起始扇区

always @(posedge clk or posedge rst) begin
    if (rst) begin
        boot_jmp     <= 8'd0;
        fs_type_0    <= 8'd0;
        fs_type_1    <= 8'd0;
        fs_type_2    <= 8'd0;
        fs_type_3    <= 8'd0;
        fs_type_4    <= 8'd0;
        bpb_is_bpb   <= 1'b0;
        mbr_sig_valid<= 1'b0;
        mbr_sig_0    <= 8'd0;
        mbr_raw_start<= 32'd0;
    end else if (state == ST_SCAN_MBR && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0   : boot_jmp  <= sd_sec_read_data;                 // 跳转指令
            10'd82  : fs_type_0 <= sd_sec_read_data;                 // "FAT32"
            10'd83  : fs_type_1 <= sd_sec_read_data;
            10'd84  : fs_type_2 <= sd_sec_read_data;
            10'd85  : fs_type_3 <= sd_sec_read_data;
            10'd86  : fs_type_4 <= sd_sec_read_data;
            10'd454 : mbr_raw_start[7:0]   <= sd_sec_read_data;
            10'd455 : mbr_raw_start[15:8]  <= sd_sec_read_data;
            10'd456 : mbr_raw_start[23:16] <= sd_sec_read_data;
            10'd457 : mbr_raw_start[31:24] <= sd_sec_read_data;
            10'd510 : mbr_sig_0 <= sd_sec_read_data;
            10'd511 : begin
                        mbr_sig_valid <= (mbr_sig_0 == 8'h55) && (sd_sec_read_data == 8'hAA);
                        // 扇区0 是 BPB：跳转指令 EB/E9 且文件系统类型字符串 "FAT32"
                        bpb_is_bpb    <= ((boot_jmp == 8'hEB) || (boot_jmp == 8'hE9)) &&
                                         (fs_type_0 == "F") && (fs_type_1 == "A") &&
                                         (fs_type_2 == "T") && (fs_type_3 == "3") &&
                                         (fs_type_4 == "2");
                     end
            default: ;
        endcase
    end
end

// 有效的分区起始扇区：
//   - 若扇区 0 就是 BPB（整卡 FAT32）→ part_start = 0
//   - 否则若为 MBR（有 0x55AA 签名）→ 用分区表项里的起始扇区
//   - 都不是（异常）→ 0，让后续 BPB 解析自然失败
wire [31:0] part_start_eff = bpb_is_bpb ? 32'd0 :
                             (mbr_sig_valid ? mbr_raw_start : 32'd0);

// ---------------------------------------------------------------------------
// BPB 字段解析：读分区内 BPB 扇区时按字节偏移抽取
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        reserved_sectors      <= 32'd0;
        num_fats              <= 32'd0;
        fat_size              <= 32'd0;
        sectors_per_cluster   <= 32'd0;
        root_cluster          <= 32'd0;
    end else if (state == ST_SCAN_BPB && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd11 : ;   // bytes per sector 低字节，固定 512，忽略
            10'd12 : ;   // bytes per sector 高字节
            10'd13 : sectors_per_cluster <= {24'd0, sd_sec_read_data};
            10'd14 : reserved_sectors[7:0]  <= sd_sec_read_data;
            10'd15 : reserved_sectors[15:8] <= sd_sec_read_data;
            10'd16 : num_fats <= {24'd0, sd_sec_read_data};
            10'd36 : fat_size[7:0]   <= sd_sec_read_data;
            10'd37 : fat_size[15:8]  <= sd_sec_read_data;
            10'd38 : fat_size[23:16] <= sd_sec_read_data;
            10'd39 : fat_size[31:24] <= sd_sec_read_data;
            10'd44 : root_cluster[7:0]   <= sd_sec_read_data;
            10'd45 : root_cluster[15:8]  <= sd_sec_read_data;
            10'd46 : root_cluster[23:16] <= sd_sec_read_data;
            10'd47 : root_cluster[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

// ---------------------------------------------------------------------------
// 目录项解析：读根目录扇区时，逐 32 字节目录项，提取 .BMP 起始簇
// ---------------------------------------------------------------------------
// 短文件名：偏移0-7 = 主名，偏移8-10 = 扩展名（BMP）
// 属性：偏移11；长文件名项属性=0x0F；目录=0x10
// 起始簇：偏移20-21 高16位，偏移26-27 低16位（均小端）
//
// 字节序（实测验证）：偏移26=低16位低字节(→[7:0])，偏移27=低16位高字节(→[15:8])，
//                     偏移20=高16位低字节(→[23:16])，偏移21=高16位高字节(→[31:24])。
// 用 rd_cnt（扇区内绝对字节偏移 0~511）定位，每个目录项占 32 字节：
//   项 i 的字段偏移 = i*32 + 字段内偏移
always @(posedge clk or posedge rst) begin
    if (rst) begin
        dir_lfn           <= 1'b0;
        dir_del           <= 1'b0;
        dir_attr_dir      <= 1'b0;
        dir_name8         <= 8'd0;
        dir_ext0          <= 8'd0;
        dir_ext1          <= 8'd0;
        dir_ext2          <= 8'd0;
        dir_start_cluster <= 32'd0;
    end else if ((state == ST_SCAN_ROOT || state == ST_SCAN_NEXT) && sd_sec_read_data_valid) begin
        // 用 rd_cnt[4:0]（0~31）表示目录项内偏移，rd_cnt[8:5] 表示项序号
        case (rd_cnt[4:0])
            // 目录项首字节（偏移 0）的三种取值
            //   0x00 = 该项及其后全部空闲，目录结束
            //   0xE5 = 该项已被删除：文件名/扩展名/属性/起始簇/文件长度都还在，
            //          只有首字节被改成 0xE5。不判这一位就会把删掉的图当有效图读，
            //          而它的簇已被释放并被新文件部分占用 → 读到"旧图头 + 新图身"
            //          的拼合画面（换图后仍显示旧图、或第一张是二合一，即此因）。
            //   其它 = 有效短文件名
            5'd0  : begin
                        dir_name8 <= sd_sec_read_data;
                        dir_del   <= (sd_sec_read_data == 8'hE5);
                    end
            5'd8  : dir_ext0  <= sd_sec_read_data;
            5'd9  : dir_ext1  <= sd_sec_read_data;
            5'd10 : dir_ext2  <= sd_sec_read_data;
            5'd11 : begin
                        dir_lfn      <= (sd_sec_read_data[3:0] == 4'hF);  // 0x0F
                        dir_attr_dir <= sd_sec_read_data[4];
                     end
            // 小端字节序：偏移20=高16位低字节，偏移21=高16位高字节
            5'd20 : dir_start_cluster[23:16] <= sd_sec_read_data;  // 高16位低字节
            5'd21 : dir_start_cluster[31:24] <= sd_sec_read_data;  // 高16位高字节
            // 小端字节序：偏移26=低16位低字节，偏移27=低16位高字节
            5'd26 : dir_start_cluster[7:0]   <= sd_sec_read_data;  // 低16位低字节
            5'd27 : dir_start_cluster[15:8]  <= sd_sec_read_data;  // 低16位高字节
            default: ;
        endcase
    end
end

// 目录项判定（放这里而不是文件末尾：ModelSim 不接受先用后声明，TD 能过，
// 但为了两个工具都能编译，统一提前声明）
wire bmp_ext_match = (dir_ext0 == "B") && (dir_ext1 == "M") && (dir_ext2 == "P");
// 必须带 !dir_del：FAT32 删除文件只把目录项首字节改成 0xE5，
// 扩展名 / 起始簇 / 文件长度一概保留；漏掉这一项，删掉的图会被当成有效图。
wire bmp_valid_entry = bmp_ext_match && !dir_lfn && !dir_attr_dir && !dir_del
                       && (dir_start_cluster != 32'd0);

// ---------------------------------------------------------------------------
// 主状态机
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        found_cnt         <= 3'd0;
        first_data_sector <= 32'd0;
        root_dir_sector   <= 32'd0;
        root_dir_scan_cnt <= 32'd0;
        root_dir_scan_max <= 32'd0;
        load_sector_latched <= 32'd0;
        img_w             <= 16'd0;
        img_h             <= 16'd0;
        img_32bpp         <= 1'b0;
        cur_clus          <= 32'd0;
        sec_in_clus       <= 32'd0;
        fat_item          <= 32'd0;
        fat_off           <= 32'd0;
        fat_cnt           <= 10'd0;
        fat_lba           <= 32'd0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        found_cnt         <= 3'd0;
        load_sector_latched <= 32'd0;
    end else if (load_abort) begin
        // 关键修复：加载中止只复位"读图"通道，不能连扫描结果一起清掉。
        // 原来 load_abort 和 !sd_init_done 共用一个分支，超时中止会把
        // scan_done 清 0，而 sd_card_bmp 侧没有任何重新扫描的路径，
        // 于是 next/auto 两个按键条件永远为假 —— 表现为"播一轮后按键全失效"。
        state               <= ST_IDLE;
        state_code          <= 4'd1;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        load_sector_latched <= 32'd0;
    end else begin
        scan_found_valid <= 1'b0;

        case (state)
            // -----------------------------------------------------------------
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done         <= 1'b0;
                    found_cnt         <= 3'd0;
                    scan_found_total  <= 3'd0;
                    root_dir_scan_cnt <= 32'd0;
                    root_dir_scan_max <= 32'd64;   // 根目录最多扫 64 扇区（32KB，足够）
                    sd_sec_read_addr  <= 32'd0;     // 先读物理扇区0 = MBR
                    state             <= ST_SCAN_MBR;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            // -----------------------------------------------------------------
            // 读 MBR（物理扇区 0），提取分区起始扇区
            ST_SCAN_MBR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // 跳到分区内 BPB（分区起始扇区 + 0）。无分区表则 part_start_eff=0，
                    // 扇区 0 本身就是 BPB，两者统一。
                    sd_sec_read_addr <= part_start_eff;
                    state            <= ST_SCAN_BPB;
                end
            end

            // -----------------------------------------------------------------
            // 读分区内 BPB 扇区
            ST_SCAN_BPB: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    // 计算数据区起始（分区内相对）与根目录物理扇区
                    first_data_sector <= reserved_sectors + (num_fats * fat_size);
                    // 根目录物理扇区 = part_start + first_data + (root_cluster-2)*spc
                    //
                    // 簇偏移走移位，别写回乘法：spc 按 FAT32 规范只能是 2 的幂，
                    // spc_shift 就是它的 log2（见上面那张查找表）。原来这里写的是
                    // * sectors_per_cluster，两个操作数都按 reg [31:0] 综合，出来是
                    // 32×32 变量乘法，凭空吃掉一笔 DSP 预算 —— 而这段读完 BPB 只跑
                    // 一次，跟每簇都跑一遍的 ST_LOAD_FAT 用同一套移位算法才合理。
                    root_dir_sector   <= part_start_eff
                                         + reserved_sectors + (num_fats * fat_size)
                                         + ((root_cluster - 32'd2) << spc_shift);
                    sd_sec_read_addr  <= part_start_eff
                                         + reserved_sectors + (num_fats * fat_size)
                                         + ((root_cluster - 32'd2) << spc_shift);
                    state             <= ST_SCAN_ROOT;
                end
            end

            // -----------------------------------------------------------------
            // 读根目录扇区（第一个扇区）
            ST_SCAN_ROOT: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                // 目录项即时判定：每读完 32 字节目录项（rd_cnt 低5位==31），
                // 若为 .BMP 文件则记录起始扇区
                if (sd_sec_read_data_valid && (rd_cnt[4:0] == 5'd31) && bmp_valid_entry
                    && (found_cnt < scan_target_count)) begin
                    scan_found_valid  <= 1'b1;
                    // 定位图片首扇区，簇偏移同样走移位
                    scan_found_sector <= part_start_eff + first_data_sector + ((dir_start_cluster - 32'd2) << spc_shift);
                    found_cnt         <= found_cnt + 3'd1;
                    scan_found_total  <= scan_found_total + 3'd1;
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    root_dir_scan_cnt <= root_dir_scan_cnt + 32'd1;
                    // 判断：找够图片 或 扫完根目录
                    if (found_cnt >= scan_target_count) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else if (root_dir_scan_cnt >= root_dir_scan_max) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                        state            <= ST_SCAN_NEXT;
                    end
                end
            end

            // -----------------------------------------------------------------
            // 读下一个根目录扇区
            ST_SCAN_NEXT: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                // 目录项即时判定（同上）
                if (sd_sec_read_data_valid && (rd_cnt[4:0] == 5'd31) && bmp_valid_entry
                    && (found_cnt < scan_target_count)) begin
                    scan_found_valid  <= 1'b1;
                    // 定位图片首扇区，簇偏移同样走移位
                    scan_found_sector <= part_start_eff + first_data_sector + ((dir_start_cluster - 32'd2) << spc_shift);
                    found_cnt         <= found_cnt + 3'd1;
                    scan_found_total  <= scan_found_total + 3'd1;
                end

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    root_dir_scan_cnt <= root_dir_scan_cnt + 32'd1;
                    if (found_cnt >= scan_target_count) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else if (root_dir_scan_cnt >= root_dir_scan_max) begin
                        scan_done <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                        state            <= ST_SCAN_NEXT;
                    end
                end
            end

            // -----------------------------------------------------------------
            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (hdr_ok) begin
                        // 锁存源图尺寸与位深，供 img_scaler 计算步进
                        img_w      <= hdr_w;
                        img_h      <= hdr_h;
                        img_32bpp  <= (bit_count == 16'd32);
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        // 由起始物理扇区反推起始簇号，并记下 FAT1 表起点：
                        //   clus   = (sector - 分区基址 - 数据区基址) / spc + 2
                        //   fat_lba = 分区基址 + 保留扇区数（FAT1 起始）
                        cur_clus    <= ((load_sector_latched - part_start_eff - first_data_sector)
                                        >> spc_shift) + 32'd2;
                        sec_in_clus <= 32'd0;
                        fat_lba     <= part_start_eff + reserved_sectors;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else if ((sectors_per_cluster != 32'd0) &&
                                 ((sec_in_clus + 32'd1) >= sectors_per_cluster)) begin
                        // 本簇读完 → 去 FAT 找下一簇。
                        // 只有 BPB 解析成功（spc != 0）才跳簇；spc == 0 说明 FAT 参数
                        // 没读到，保持旧的"扇区号线性 +1"行为，不会更糟。
                        // 表项在 FAT 扇区内的偏移 = (簇号 & 127) × 4
                        // 表项所在扇区 = fat_lba + (簇号 >> 7)
                        fat_off          <= {23'd0, cur_clus[6:0], 2'b00};
                        fat_cnt          <= 10'd0;
                        sd_sec_read_addr <= fat_lba + (cur_clus >> 7);
                        state            <= ST_LOAD_FAT;
                    end else begin
                        sec_in_clus      <= sec_in_clus + 32'd1;
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            // -----------------------------------------------------------------
            // 读 FAT 表项 → 下一簇（碎片化支持）
            //   只取目标 4 字节，其余字节丢弃；512 字节读完后再跳。
            // -----------------------------------------------------------------
            ST_LOAD_FAT: begin
                state_code  <= 4'd5;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_data_valid) begin
                    if (fat_cnt == fat_off)
                        fat_item[7:0]   <= sd_sec_read_data;
                    else if (fat_cnt == (fat_off + 32'd1))
                        fat_item[15:8]  <= sd_sec_read_data;
                    else if (fat_cnt == (fat_off + 32'd2))
                        fat_item[23:16] <= sd_sec_read_data;
                    else if (fat_cnt == (fat_off + 32'd3))
                        fat_item[31:24] <= sd_sec_read_data;
                    fat_cnt <= fat_cnt + 10'd1;
                end
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if ((fat_item_n[27:0] >= 28'hFFFFFF8) || (fat_item_n[27:0] < 28'd2)) begin
                        // 簇链提前结束（文件长度与 FAT 不符）：直接收工，避免死循环。
                        // 注意：剩余行不会补黑，img_scaler 的 row_avail 会一直等不到
                        // 数据，直到 sd_card_bmp 的 8s 加载超时后判失败并跳过这张图。
                        state <= ST_IDLE;
                    end else begin
                        cur_clus         <= {4'd0, fat_item_n[27:0]};
                        sec_in_clus      <= 32'd0;
                        sd_sec_read_addr <= part_start_eff + first_data_sector
                                          + (({4'd0, fat_item_n[27:0]} - 32'd2) << spc_shift);
                        state            <= ST_LOAD_DATA;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

// ---------------------------------------------------------------------------
// 目录项即时判定：在根目录扇区字节流中，每读完一个 32 字节目录项，
// 若为 .BMP 文件，则记录起始扇区并置 scan_found_valid。
//
// 判定时机：rd_cnt 低 5 位 == 31（当前目录项最后一个字节）且 data_valid 时。
// 此时 dir_ext0/1/2、dir_lfn、dir_attr_dir、dir_start_cluster 已就绪。
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 加载阶段的 BMP 头解析（复用原逻辑）
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if ((state == ST_LOAD_HDR) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

// ---------------------------------------------------------------------------
// 像素数据加载（复用原逻辑）
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if ((state == ST_LOAD_DATA) || (state == ST_LOAD_FAT)) begin
        // 查 FAT 期间必须保持计数值：若只在 ST_LOAD_DATA 保持、其它状态清零，
        // 加了 ST_LOAD_FAT 后每跳一次簇就把已读字节数抹掉，
        // 导致永远到不了 file_len，无限跳簇（死循环）。
        if ((state == ST_LOAD_DATA) && sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

// ---------------------------------------------------------------------------
// 源像素流输出（支持任意分辨率 + 跳过行尾对齐填充）
//
// 早期实现按"每 3 字节一个像素"输出，隐含"行字节数 = width×3 且恰为 4 的倍数"
// 的假设（640×3=1920 成立，其它宽度大多不成立），行尾填充字节会被当成像素
// 拼进去，整幅图斜向错位。现改为按行内字节偏移推进：
//   row_byte 在 [0, w×bpp/8) 内是像素字节，其后的填充字节直接丢弃；
//   行末（row_byte == stride-1）复位、行号 +1。
// 同时输出 pix_x（本行像素序号），缩放模块据此判断行尾、做水平插值。
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pix_de   <= 1'b0;
        pix_data <= 24'd0;
        pix_x    <= 16'd0;
        px_acc   <= 24'd0;
        px_cnt   <= 16'd0;
        py_cnt   <= 16'd0;
        row_byte <= 32'd0;
        byte_ph  <= 2'd0;
    end else if ((state != ST_LOAD_DATA) && (state != ST_LOAD_FAT)) begin
        // ST_LOAD_FAT 必须排除：查 FAT 只是"借"读通道，属加载过程的一部分。
        // 若按 state != ST_LOAD_DATA 就清零，每跳一次簇便把 row_byte / byte_ph /
        // px_cnt 归零 → 行从中间断开、字节相位错位，表现为"每 32KB 又从第 0 列
        // 开始画"，整幅图糊成一团（加簇链支持时踩到，tb_frag 逐像素比对抓出）。
        pix_de   <= 1'b0;
        px_cnt   <= 16'd0;
        py_cnt   <= 16'd0;
        row_byte <= 32'd0;
        byte_ph  <= 2'd0;
    end else begin
        pix_de <= 1'b0;
        if (bmp_data_valid) begin
            if (row_byte < hdr_row_bytes) begin
                // 有效像素字节：BMP 按 B,G,R(,A) 存，这里拼成 {R,G,B}
                // 触发输出的那一拍必须把当前字节一起拼进去：px_acc 是非阻塞赋值，
                // 读它在输出拍的旧值会少最后 1 字节，整幅图颜色通道错位 1 字节。
                if (img_32bpp) begin
                    case (byte_ph)
                        2'd0: px_acc[7:0]   <= sd_sec_read_data;
                        2'd1: px_acc[15:8]  <= sd_sec_read_data;
                        2'd2: px_acc[23:16] <= sd_sec_read_data;
                        default: ;                   // 第 4 字节（alpha）丢弃
                    endcase
                    if (byte_ph == 2'd3) begin
                        byte_ph  <= 2'd0;
                        pix_de   <= 1'b1;
                        pix_data <= {px_acc[23:16], px_acc[15:8], px_acc[7:0]};
                        pix_x    <= px_cnt;
                        px_cnt   <= px_cnt + 16'd1;
                    end else begin
                        byte_ph <= byte_ph + 2'd1;
                    end
                end else begin
                    case (byte_ph)
                        2'd0: px_acc[7:0]  <= sd_sec_read_data;
                        2'd1: px_acc[15:8] <= sd_sec_read_data;
                        default: ;                   // byte_ph == 2，R 在下面直接拼
                    endcase
                    if (byte_ph == 2'd2) begin
                        byte_ph  <= 2'd0;
                        pix_de   <= 1'b1;
                        pix_data <= {sd_sec_read_data, px_acc[15:8], px_acc[7:0]};
                        pix_x    <= px_cnt;
                        px_cnt   <= px_cnt + 16'd1;
                    end else begin
                        byte_ph <= byte_ph + 2'd1;
                    end
                end
            end
            // 行内字节偏移推进（含填充字节）；行末复位
            if (row_byte == (hdr_row_stride - 32'd1)) begin
                row_byte <= 32'd0;
                px_cnt   <= 16'd0;                   // 行末清零（覆盖上面的 +1）
                if (py_cnt != img_h)
                    py_cnt <= py_cnt + 16'd1;
            end else begin
                row_byte <= row_byte + 32'd1;
            end
        end
    end
end

endmodule
