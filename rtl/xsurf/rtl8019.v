//
// RTL8019AS (16-bit only) implementation for SPI bus
// Copyright (c) 2026 Eugene Azarov <rusjaz@gmail.com>
//

module rtl8019
(
	input             clk,
	input             rst,
	input             rd,
	input             wr,
	input      [4:0]  addr,
	input      [15:0] din,
	output reg [15:0] dout,
	output            irq,

	output     [31:0] status,

	input             tx_begin,
	input             tx_strobe,
	output reg  [7:0] tx_byte,

	input             rx_begin,
	input             rx_strobe,
	input       [7:0] rx_byte,

	input             mac_begin,
	input             mac_strobe,
	input       [7:0] mac_byte
);

// tx_ready[17], rx_ready[16], tx_count[15:0]
assign status = { 8'ha8, 6'h00, tx_ready, rx_ready, 5'h00, tbcr };

// NE2000 registers
reg [7:0]  cr;             // command register
reg [7:0]  isr;            // interrupt service register
reg [7:0]  imr;            // interrupt mask register
reg [7:0]  rcr;            // receiver control register
reg [7:0]  rsr;            // receiver status register
reg [7:0]  tcr;            // transmitter control register
reg [7:0]  tsr;            // transmitter status register
reg [7:0]  curr;           // current page register
reg [7:0]  bnry;           // boundary page
reg [7:0]  clda;           // current local dma page register
reg [7:0]  pstart;         // rx buffer ring start page
reg [7:0]  pstop;          // rx buffer ring stop page
reg [15:1] rsar;           // remote start address register
reg [15:1] rbcr;           // remote byte count register
reg [3:0]  tpsr;           // transmit page start address
reg [10:0] tbcr;           // transmitter byte count register

wire stop = cr[0];         // stop mode
wire txp  = cr[2];         // transmit packet toggle
wire [1:0] ps = cr[7:6];   // register page select
wire [1:0] lb = tcr[2:1];  // loopback mode select
wire mon = rcr[5];         // monitor mode

wire dma_port = (addr[4:3] == 2'b10); // remote DMA ports ($10 - $17)
wire rst_port = (addr[4:3] == 2'b11); // reset ports ($18 - $1F)

// rx/tx buffers
reg [15:0] rx_buffer[2047:0]; // 2-4 ethernet frames
reg [15:0] tx_buffer[2047:0]; // 2 ethernet frames (ping-pong)

// control signals
reg rd_d, wr_d;

always @(posedge clk) begin
	rd_d <= rd;
	wr_d <= wr;
end

wire rd_ne = ~rd & rd_d;
wire wr_ne = ~wr & wr_d;

reg rx_begin_d;
reg mac_begin_d;

// same clock domain
always @(posedge clk) begin
	rx_begin_d <= rx_begin;
	mac_begin_d <= mac_begin;
end

wire rx_start = rx_begin & ~rx_begin_d;
wire rx_stop = ~rx_begin &  rx_begin_d;
wire mac_start = mac_begin & ~mac_begin_d;

reg [1:0] tx_begin_sr;
reg [1:0] tx_strobe_sr;

// belongs to SPI clock domain
always @(posedge clk) begin
	if (rst) begin
		tx_begin_sr  <= 2'd0;
		tx_strobe_sr <= 2'd0;
	end else begin
		tx_begin_sr  <= { tx_begin_sr[0],  tx_begin  };
		tx_strobe_sr <= { tx_strobe_sr[0], tx_strobe };
	end
end

wire tx_start = tx_begin_sr[0] & ~tx_begin_sr[1];
wire tx_done = ~tx_begin_sr[0] &  tx_begin_sr[1];
wire tx_strobe_pe = tx_strobe_sr[0] & ~tx_strobe_sr[1];

// reset
reg reset = 1'b0;

always @(posedge clk) begin
	if (rst) begin
		reset <= 1'b0;
	end else if (rd_ne) begin
		if (rst_port) reset <= 1'b1;
	end else if (wr_ne) begin
		if (rst_port) reset <= 1'b0;
	end
end

// syncing i/o controller rx frame traffic
wire [3:0] used = (curr - bnry);
wire full = (used >= 10);

reg tx_ready;
reg rx_ready;

always @(posedge clk) begin
	tx_ready <= (~stop & ~is_loop & txp);
	rx_ready <= (~stop & ~is_loop & ~mon & ~full);
end

// set local MAC address
reg [7:0] mac [5:0];
reg [2:0] mac_cnt;

always @(posedge clk) begin
	if (mac_start)
		mac_cnt <= 0;
	else if (mac_strobe) begin
		if (mac_cnt < 6) begin
			mac[mac_cnt] <= mac_byte;
			mac_cnt <= mac_cnt + 3'd1;
		end
	end
end

reg  [7:0] reg_do;
reg [15:0] rom_do;

reg [15:0] rx_buffer_do;
reg [15:0] tx_buffer_do;

// PROM read (16-bit)
always @(*) begin
	case (rsar[3:1])
		3'h0: rom_do = { mac[0], mac[0] };
		3'h1: rom_do = { mac[1], mac[1] };
		3'h2: rom_do = { mac[2], mac[2] };
		3'h3: rom_do = { mac[3], mac[3] };
		3'h4: rom_do = { mac[4], mac[4] };
		3'h5: rom_do = { mac[5], mac[5] };
		3'h7: rom_do = 16'h5757;
		default: rom_do = 16'd0;
	endcase
end

// register read
always @(*) begin
	reg_do = 8'h00;
	// cr is available on all pages
	if (addr == 0) begin
		reg_do = cr;
	end
	// page 0
	else if (ps == 0) begin
		case (addr)
			5'h03: reg_do = bnry;
			5'h04: reg_do = tsr;
			5'h07: reg_do = isr;
			5'h08: reg_do = { rsar[7:1], 1'b0 };
			5'h09: reg_do = rsar[15:8];
			5'h0a: reg_do = { rbcr[7:1], 1'b0 };
			5'h0b: reg_do = rbcr[15:8];
			5'h0c: reg_do = rsr;
			default: ;
		endcase
	end
	// page 1
	else if (ps == 1) begin
		if (addr == 7) reg_do = curr;
	end
end

wire is_loop = (lb != 2'b00);
wire is_prom = (rsar[15:8] == 8'h00);

wire [15:0] dma_do = (is_prom) ? rom_do
	: (is_loop) ? tx_buffer_do
	: rx_buffer_do;

// CPU read
always @(*) begin
	if (rd) begin
		dout = dma_port ? dma_do : { reg_do, reg_do };
	end else begin
		dout = 16'd0;
	end
end

wire [7:0] next_curr = ((curr + 8'd1) == pstop) ? pstart : (curr + 8'd1);
wire [7:0] next_clda = ((clda + 8'd1) == pstop) ? pstart : (clda + 8'd1);

reg rx_fin;
reg rx_fin_d;

wire rx_done = ~rx_fin & rx_fin_d;

reg [11:0] rx_addr;
reg [10:0] rx_length;
reg  [7:0] lo_byte;

// local DMA frame receiver
always @(posedge clk) begin
	if (rst) begin
		rsr <= 8'h00;
		rx_fin <= 1'b0;
		rx_fin_d <= 1'b0;
	end else begin
		rx_fin_d <= rx_fin;
		if (rx_start) begin
			rsr <= 8'h00;
			// +4 for crc
			rx_length <= 11'd4;
			// +4 for header
			rx_addr <= { curr[3:0], 8'd4 };
			// first page already used
			clda <= next_curr;
		end else if (rx_strobe && !rsr[2]) begin
			if (rx_addr[0] == 1'b0) begin
				lo_byte <= rx_byte;
			end else begin
				rx_buffer[rx_addr[11:1]] <= { rx_byte, lo_byte };
			end
			rx_length <= rx_length + 11'd1;
			// write past pstop boundary
			rx_addr <= rx_addr + 12'd1;
			// full page count
			if (rx_addr[7:0] == 255)
				clda <= next_clda;
			// check max frame limit
			if (rx_length == 1518)
				rsr[2] <= 1'b1; // FAE
		end else if (rx_stop) begin
			if (rx_addr[0] == 1'b1) begin
				// last odd byte
				rx_buffer[rx_addr[11:1]] <= { 8'h00, lo_byte };
			end
			// prepare header write
			rx_addr <= { curr[3:0], 8'd0 };
			rsr[0] <= ~rsr[2]; // PRX
			rx_fin <= 1'b1;
		end else if (rx_fin) begin
			// writing frame header
			if (rx_addr[1] == 1'b0) begin
				rx_buffer[rx_addr[11:1]] <= { clda, rsr };
				rx_addr[1] <= 1'b1;
			end else begin
				rx_buffer[rx_addr[11:1]] <= { 5'h00, rx_length[10:8], rx_length[7:0] };
				if (rx_length < 64) begin
					// allow/drop runt frames
					rx_fin_d <= rcr[1];
				end
				rx_fin <= 1'b0;
			end
		end
	end
end

reg  [11:0] tx_addr;
wire [11:1] tx_r_addr = (is_loop) ? rsar[11:1] : tx_addr[11:1];
wire tx_byte_sel = tx_addr[0];

wire wrong_tx = (tbcr == 0 || tbcr > 1514) && txp;

// local DMA frame transmitter
always @(posedge clk) begin
	tx_byte <= (tx_byte_sel) ? tx_buffer_do[15:8] : tx_buffer_do[7:0];
	if (rst) begin
		tsr <= 8'h00;
	end else begin
		if (wrong_tx) begin
			// avoid sending invalid frames
			tsr <= 8'h20; // FU
		end else if (tx_start) begin
			tsr <= 8'h00;
			tx_addr <= { tpsr, 8'd0 };
		end else if (tx_strobe_pe) begin
			tx_addr <= tx_addr + 12'd1;
		end else if (tx_done) begin
			tsr[0] <= 1'b1; // PTX
		end
	end
end

// remote DMA data reader
always @(posedge clk) begin
	rx_buffer_do <= rx_buffer[rsar[11:1]];
end

// remote DMA data reader
always @(posedge clk) begin
	tx_buffer_do <= tx_buffer[tx_r_addr];
end

// remote DMA data writer
always @(posedge clk) begin
	if (wr_ne) begin
		if (dma_port && !rbcr_is_0) begin
			tx_buffer[rsar[11:1]] <= din;
		end
	end
end

wire rbcr_is_0 = (rbcr == 15'd0);
wire rbcr_is_1 = (rbcr == 15'd1);

// CPU write
always @(posedge clk) begin
	if (rst || reset) begin
		cr   <= 8'h21;    // ABORT, STP
		rcr  <= 8'h20;    // MON
		tcr  <= 8'h02;    // LB1
		isr  <= 8'h80;    // RST
		imr  <= 8'h00;
		rbcr <= 15'h3828; // RTL8019 Id (0x7050)
	end else begin
		if (wr_ne) begin
			if (!dma_port) begin
				// cr is available on all pages
				if (addr == 0) begin
					{ cr[7:3], cr[1:0] } <= { din[7:3], din[1:0] };
					if (!din[0]) begin
						// not stop
						if (cr[0]) begin
							isr[7] <= 1'b0; // RST
						end
						if (din[2]) begin
							cr[2] <= 1'b1;  // TXP
						end
						// dummy read or write
						if (rbcr_is_0 && (din[3] || din[4])) begin
							isr[6] <= 1'b1; // RDC
						end
 					end
				end
				// page 0
				else if (ps == 0) begin
					case (addr)
						5'h01: pstart <= din[7:0];
						5'h02: pstop <= din[7:0];
						5'h03: bnry  <= din[7:0];
						5'h04: tpsr  <= din[3:0];
						5'h05: tbcr[7:0]  <= din[7:0];
						5'h06: tbcr[10:8] <= din[2:0];
						5'h07: isr <= isr & ~din[7:0]; // write-1-to-clear
						5'h08: rsar[7:1]  <= din[7:1];
						5'h09: rsar[15:8] <= din[7:0];
						5'h0a: rbcr[7:1]  <= din[7:1];
						5'h0b: rbcr[15:8] <= din[7:0];
						5'h0c: rcr <= din[7:0];
						5'h0d: tcr <= din[7:0];
						5'h0f: imr <= din[7:0];
						default: ;
					endcase
				end
				// page 1
				else if (ps == 1) begin
					if (addr == 7) curr <= din[7:0];
				end
			end
		end
		// remote dma operation
		if (rd_ne || wr_ne) begin
			if (dma_port && !rbcr_is_0) begin
				rsar <= rsar + 15'd1;
				rbcr <= rbcr - 15'd1;
				if (rbcr_is_1)
					isr[6] <= 1'b1; // RDC
			end
		end
		// incoming frame received
		if (rx_done) begin
			isr[0] <= 1'b1; // PRX
			curr <= clda;
		end
		// outgoing frame transmitted
		if (tx_done) begin
			isr[1] <= 1'b1; // PTX
			cr[2] <= 1'b0;  // TXP
		end
		// transmitting wrong-length frame
		if (wrong_tx) begin
			isr[3] <= 1'b1; // TXE
			cr[2] <= 1'b0;  // TXP
		end
	end
end

assign irq = (|(isr & imr) & ~reset);

endmodule
