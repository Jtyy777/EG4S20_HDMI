// ============================================================================
// sd_card_cmd —— SD 卡 SPI 协议层：命令打包、R1 响应判别、扇区读写搬运
// ----------------------------------------------------------------------------
// 分工：sd_card_sec_read_write 决定「发哪条命令」，本模块负责「怎么发、
// 怎么收」；位时序交给 spi_master。三者的连接在 sd_card_top 里。
//
// 一、上电初始时钟
//   S_IDLE → S_INIT：不发命令，先空打 11 个 0xFF（= 88 个 SPI 时钟，满足
//   SD 规范「上电至少 74 个时钟」的要求），让卡进入 SPI 模式。
//
// 二、一条命令的完整时序（S_CMD_PRE → S_CMD → 可选 S_CMD_DATA）
//   · S_CMD_PRE：先发 1 个 0xFF 提供时钟，此时 CS 还是高的；
//   · S_CMD：把 CS 拉低，逐字节发 6 个命令字节：
//       cmd[47:40] | 0x40   ← 命令号，最高两位固定为 01（起始位+传输位）
//       cmd[39:0]           ← 32 位参数 + CRC
//     后面继续发 0xFF 换时钟，同时盯 MISO：
//       · 收到最高位为 0 的字节 = 收到合法 R1，若等于 cmd_r1 则成功；
//         若不等（且最高位为 0）说明卡明确报了错，直接进 S_ERR 置
//         cmd_req_error —— 这样上层能立刻知道命令失败，不必等超时；
//       · 一直等不到就靠 byte_cnt 数到 0xFFFF 放弃，防止死循环。
//   · S_CMD_DATA：给 CMD8 这类需要读 4 字节 R7 响应的命令用。
//
// 三、读一个扇区（CMD17 之后）
//   S_READ_WAIT 反复发 0xFF 找数据起始 token 0xFE，找到即进入 S_READ。
//   然后整整 512 拍，每拍收一个字节并给出 block_read_valid —— 注意 token
//   是在 S_READ_WAIT 那一拍就被吃掉的，所以 S_READ 的第 0 拍收到的就是
//   第一个数据字节，不会把 0xFE 混进数据里。收满 512 字节后再收 2 字节
//   CRC（不计入有效数据）才进 S_READ_ACK。
//
// 四、读超时与恢复（S_RECOVER）
//   S_READ_WAIT 等 token 最长等 RD_WAIT_TIMEOUT（24'd312500 拍，SPI 25MHz
//   下约 100ms，正好是 SD 规范允许的读延迟上限）。等满就不装了：
//   拉高 CS + 空发 RECOVER_BYTES 个 0xFF，让卡放弃当前扇区、复位自身传输
//   状态，然后才回 S_READ_ACK 并置 cmd_req_error。
//   为什么必须先拉 CS 而不能直接回 S_READ_ACK——卡这时候还在往 MISO 上吐
//   这个扇区的残留字节，FPGA 若立刻发下一条命令，收到的第一个字节是残留
//   数据而不是 R1，S_CMD 就会判错并无限重试，整条读通道卡死（现象是画面
//   停住、按键也没反应）。
//
// 五、写一个扇区（CMD24 之后）
//   S_WRITE_TOKEN 发 0xFE 起始 token → S_WRITE_DATA_0/_1 交替搬 512 字节
//   （block_write_data_rd 在 _0 拍置位，表示「下一拍要下一字节」）→
//   S_WRITE_CRC 发 2 字节 CRC → S_WRITE_SUC 收数据响应
//   （data_recv[4:0] == 5'b00101 表示卡已接收）→ S_WRITE_BUSY 等 MISO 回到
//   0xFF（卡写完了）→ S_WRITE_ACK。这一整套是 SD 规范规定的顺序，少一步
//   就会出现「写进去读不回来」。
//
// 六、握手约定
//   cmd_req / block_read_req / block_write_req 都是「保持型」：请求方拉高后
//   一直等到对应的 _ack 才撤。_ack 是状态机的瞬时状态，只持续一拍。
// ============================================================================

module sd_card_cmd(
	input                       sys_clk,
	input                       rst,
	input[15:0]                 spi_clk_div,                  //SPI module clock division parameter
	input                       cmd_req,                      //SD card command request
	output                      cmd_req_ack,                  //SD card command request response
	output reg                  cmd_req_error,                //SD card command request error
	input[47:0]                 cmd,                          //SD card command
	input[7:0]                  cmd_r1,                       //SD card expect response
	input[15:0]                 cmd_data_len,                 //SD card command read data length
	input                       block_read_req,               //SD card sector data read request
	output reg                  block_read_valid,             //SD card sector data read data valid
	output reg[7:0]             block_read_data,              //SD card sector data read data
	output                      block_read_req_ack,           //SD card sector data read response
	input                       block_write_req,              //SD card sector data write request
	input[7:0]                  block_write_data,             //SD card sector data write data next clock is valid
	output                      block_write_data_rd,          //SD card sector data write data
	output                      block_write_req_ack,          //SD card sector data write response
	output                      nCS_ctrl,                     //SPI module chip select control
	output reg[15:0]            clk_div,
	output reg                  spi_wr_req,                   //SPI module data sending request
	input                       spi_wr_ack,                   //SPI module data request response
	output[7:0]                 spi_data_in,                  //SPI module send data
	input[7:0]                  spi_data_out                  //SPI module data returned
);
parameter S_IDLE         = 0;
parameter S_WAIT         = 1;
parameter S_INIT         = 2;
parameter S_CMD_PRE      = 3;
parameter S_CMD          = 4;
parameter S_CMD_DATA     = 5;
parameter S_READ_WAIT    = 6;
parameter S_READ         = 7;
parameter S_READ_ACK     = 8;
parameter S_WRITE_TOKEN  = 9;
parameter S_WRITE_DATA_0 = 10;
parameter S_WRITE_DATA_1 = 11;
parameter S_WRITE_CRC    = 12;
parameter S_WRITE_SUC    = 13;
parameter S_WRITE_BUSY   = 14;
parameter S_WRITE_ACK    = 15;
parameter S_ERR          = 16;
parameter S_END          = 17;
parameter S_RECOVER      = 18;    // 读超时后的复位恢复：拉高 CS、空发若干字节

reg[4:0]                      state;
reg                           CS_reg;
reg[15:0]                     byte_cnt;
reg[7:0]                      send_data;
wire[7:0]                     data_recv;
reg[9:0]                      wr_data_cnt;
reg[23:0]                     rd_wait_cnt;      // 等数据 token(0xFE) 的超时计数
reg[4:0]                      recover_cnt;      // 超时后拉高 CS 的节拍计数
// 超时窗口：SPI=25MHz 时 1 字节 = 320ns，312500 字节约 100ms，
// 正好覆盖 SD 规范允许的读访问延迟上限。
localparam [23:0] RD_WAIT_TIMEOUT = 24'd312_500;
// 超时后拉高 CS 并空发 8 字节，让卡放弃当前扇区的剩余数据、重新同步。
localparam [4:0]  RECOVER_BYTES   = 5'd8;

assign cmd_req_ack = (state == S_END);
assign block_read_req_ack = (state == S_READ_ACK);
assign block_write_req_ack= (state == S_WRITE_ACK);
assign block_write_data_rd = (state == S_WRITE_DATA_0);
assign spi_data_in = send_data;
assign data_recv = spi_data_out;
assign nCS_ctrl = CS_reg;
always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		CS_reg <= 1'b1;
		spi_wr_req <= 1'b0;
		byte_cnt <= 16'd0;
		clk_div <= 16'd0;
		send_data <= 8'hff;
		state <= S_IDLE;
		cmd_req_error <= 1'b0;
		wr_data_cnt <= 10'd0;
		rd_wait_cnt <= 24'd0;
		recover_cnt <= 5'd0;
	end
	else
		case(state)
			S_IDLE:
			begin
				state <= S_INIT;
				clk_div <= spi_clk_div;
				CS_reg <= 1'b1;
			end
			S_INIT:
			begin
				//上电空打 11 个 0xFF = 88 个 SPI 时钟，满足规范要求的「至少 74 拍」
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt >= 16'd10)
					begin
						byte_cnt <= 16'd0;
						spi_wr_req <= 1'b0;
						state <= S_WAIT;
					end
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WAIT:
			begin
				cmd_req_error <= 1'b0;
				wr_data_cnt <= 10'd0;
				rd_wait_cnt <= 24'd0;
				recover_cnt <= 5'd0;
				//wait for  instruction
				if(cmd_req == 1'b1)
					state <= S_CMD_PRE;
				else if(block_read_req == 1'b1)
					state <= S_READ_WAIT;
				else if(block_write_req == 1'b1)
					state <= S_WRITE_TOKEN;
				clk_div <= spi_clk_div;
			end
			S_CMD_PRE:
			begin
				//before sending a command, send an byte 'ff',provide some clocks
				if(spi_wr_ack == 1'b1)
				begin
					state <= S_CMD;
					spi_wr_req <= 1'b0;
					byte_cnt <= 16'd0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					CS_reg <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_CMD:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					// ① byte_cnt 数到 0xFFFF 说明一直没等到 R1，放弃；
					// ② 收到的是「合法 R1（最高位为 0）但和期望值不同」，说明卡
					//    明确报错了，直接判失败，不必继续等。
					if((byte_cnt == 16'hffff) || (data_recv != cmd_r1 && data_recv[7] == 1'b0))
					begin
						state <= S_ERR;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					// 收到期望的 R1：命令成功
					else if(data_recv == cmd_r1)
					begin
						spi_wr_req <= 1'b0;
						if(cmd_data_len != 16'd0)
						begin
							state <= S_CMD_DATA;
							byte_cnt <= 16'd0;
						end
						else
						begin
							state <= S_END;
							byte_cnt <= 16'd0;
						end
					end
					else
						byte_cnt <=  byte_cnt + 16'd1;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					CS_reg <= 1'b0;
					// 命令号要或上 0x40 —— 最高两位固定为 01（起始位 + 传输位），
					// 这是 SD 命令帧的格式要求，漏了卡不会理你。
					if(byte_cnt == 16'd0)
						send_data <= (cmd[47:40] | 8'h40);
					else if(byte_cnt == 16'd1)
						send_data <= cmd[39:32];
					else if(byte_cnt == 16'd2)
						send_data <= cmd[31:24];
					else if(byte_cnt == 16'd3)
						send_data <= cmd[23:16];
					else if(byte_cnt == 16'd4)
						send_data <= cmd[15:8];
					else if(byte_cnt == 16'd5)
						send_data <= cmd[7:0];
					else
						send_data <= 8'hff;
				end
			end
			S_CMD_DATA:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == cmd_data_len - 16'd1)
					begin
						state <= S_END;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_READ_WAIT:
			begin
				if(spi_wr_ack == 1'b1 && data_recv == 8'hfe)
				begin
					spi_wr_req <= 1'b0;
					state <= S_READ;
					byte_cnt <= 16'd0;
					rd_wait_cnt <= 24'd0;
				end
				// 关键修复：原来这里只等 token，没有超时。一旦 SD 卡连续读久了
				// 不回 0xFE（卡忙 / 读错误 / CRC 错），状态机就永久停在此处。
				// 现在等满 RD_WAIT_TIMEOUT（约 100ms）就进入恢复流程。
				else if(spi_wr_ack == 1'b1 && rd_wait_cnt >= RD_WAIT_TIMEOUT)
				begin
					// 重要：不能直接跳 S_READ_ACK 假装读成功。卡此刻还在发这个
					// 扇区剩下的字节，FPGA 若立刻发下一条 CMD17，收到的第一个字节
					// 是残留数据而不是 R1，S_CMD 会判错并无限重试 —— 整条读通道
					// 死锁，表现就是"卡死在第一张、按键全无反应"。
					// 必须先拉高 CS 让卡复位数据流，再回 S_READ_ACK。
					spi_wr_req  <= 1'b1;
					send_data   <= 8'hff;
					CS_reg      <= 1'b1;
					recover_cnt <= 5'd0;
					rd_wait_cnt <= 24'd0;
					state       <= S_RECOVER;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
					if(spi_wr_ack == 1'b1)
						rd_wait_cnt <= rd_wait_cnt + 24'd1;
				end
			end
			S_READ:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					// 0~511 是 512 个数据字节，512、513 是两个 CRC 字节（丢弃）。
					// token 0xFE 已在 S_READ_WAIT 那一拍收走，所以这里第 0 拍
					// 收到的就是第一个数据字节。
					if(byte_cnt == 16'd513)
					begin
						state <= S_READ_ACK;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WRITE_TOKEN:
				if(spi_wr_ack == 1'b1)
				begin
					state <= S_WRITE_DATA_0;
					spi_wr_req <= 1'b0;
					byte_cnt <= 16'd0;  
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hfe;
				end
			S_WRITE_DATA_0:
			begin
				state <= S_WRITE_DATA_1;
				wr_data_cnt <= wr_data_cnt + 10'd1;
			end
			S_WRITE_DATA_1:
			begin
				if(spi_wr_ack == 1'b1 && wr_data_cnt == 10'd512)
				begin
					state <= S_WRITE_CRC;
					spi_wr_req <= 1'b0;
				end
				else if(spi_wr_ack == 1'b1)
				begin
					state <= S_WRITE_DATA_0;
					spi_wr_req <= 1'b0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= block_write_data;
				end
			end
			S_WRITE_CRC:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == 16'd1)
					begin
						state <= S_WRITE_SUC;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WRITE_SUC :
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(data_recv[4:0] == 5'b00101)
					begin
						state <= S_WRITE_BUSY;
						spi_wr_req <= 1'b0;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WRITE_BUSY :
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(data_recv == 8'hff)
					begin
						state <= S_WRITE_ACK;
						spi_wr_req <= 1'b0;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_RECOVER:
			begin
				// CS 保持高电平，卡在这段时间内会丢弃未发完的扇区数据、
				// 复位自身的传输状态；同时继续打 SPI 时钟让数据彻底排空。
				CS_reg    <= 1'b1;
				send_data <= 8'hff;
				if(spi_wr_ack == 1'b1)
				begin
					if(recover_cnt >= RECOVER_BYTES - 5'd1)
					begin
						spi_wr_req    <= 1'b0;
						cmd_req_error <= 1'b1;
						byte_cnt      <= 16'd0;
						recover_cnt   <= 5'd0;
						state         <= S_READ_ACK;
					end
					else
						recover_cnt <= recover_cnt + 5'd1;
				end
				else
					spi_wr_req <= 1'b1;
			end
			S_ERR:
			begin
				state <= S_END;
				cmd_req_error <= 1'b1;
			end
			S_READ_ACK,S_WRITE_ACK,S_END:
			begin
				state <= S_WAIT;
			end
			default:
				state <= S_IDLE;
		endcase
end
always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
		block_read_valid <= 1'b0;
	else if(state == S_READ && byte_cnt < 16'd512)
		block_read_valid <= spi_wr_ack;
	else
		block_read_valid <= 1'b0;
end

always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
		block_read_data <= 8'd0;
	else if(state == S_READ && spi_wr_ack == 1'b1)
		block_read_data <= data_recv;
end

endmodule