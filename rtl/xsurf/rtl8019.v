//
// RTL8019AS (16-bit LE) implementation for SPI bus
// Copyright (c) 2026 Eugene Azarov <rusjaz@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-only
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
	output reg        irq,

	output     [31:0] status,

	input             tx_rd,
	input             tx_strobe,
	output reg  [7:0] tx_byte,

	input             rx_wr,
	input             rx_strobe,
	input       [7:0] rx_byte,

	input             mac_wr,
	input             mac_strobe,
	input       [7:0] mac_byte
);

// rst[20], irq[19], is_full[18], tx_ready[17], rx_ready[16], tx_count[15:0]
assign status = { 8'ha8, 3'h00, isr[7], irq, full, tx_ready, rx_ready, 5'h00, tbcr };

// NE2000 registers
reg [7:0]  cr;             // command register
reg [7:0]  isr;            // interrupt service register
reg [6:0]  imr;            // interrupt mask register
reg [5:0]  rcr;            // receiver control register
reg [7:0]  rsr;            // receiver status register
reg [4:0]  tcr;            // transmitter control register
reg [7:0]  tsr;            // transmitter status register
reg [7:0]  curr;           // current page register
reg [7:0]  bnry;           // boundary page
reg [7:0]  clda;           // current local dma page register
reg [7:0]  pstart;         // rx buffer ring start page
reg [7:0]  pstop;          // rx buffer ring stop page
reg [15:1] rsar;           // remote start address register
reg [10:0] rbcr;           // remote byte count register
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

// cpu signals
reg rd_d, wr_d;

always @(posedge clk) begin
	rd_d <= rd;
	wr_d <= wr;
end

wire rd_ne = ~rd & rd_d;
wire wr_ne = ~wr & wr_d;

// spi bus signals
reg tx_rd_d, rx_wr_d;

always @(posedge clk) begin
	tx_rd_d <= tx_rd;
	rx_wr_d <= rx_wr;
end

wire tx_start =  tx_rd & ~tx_rd_d;
wire tx_done  = ~tx_rd &  tx_rd_d;
wire rx_start =  rx_wr & ~rx_wr_d;
wire rx_stop  = ~rx_wr &  rx_wr_d;

// reset trigger
reg reset;

always @(posedge clk) begin
	if (rst) begin
		reset <= 1'b0;
	end else if (rd_ne) begin
		if (rst_port) reset <= 1'b1;
	end else if (wr_ne) begin
		if (rst_port) reset <= 1'b0;
	end
end

// check whether rx ring buffer is full
wire [7:0] used = (curr >= bnry)
	? (curr - bnry) : ((pstop - bnry) + (curr - pstart));
wire full = (used >= 10);

// syncing i/o controller rx frame traffic
reg tx_ready;
reg rx_ready;

always @(posedge clk) begin
	tx_ready <= (~stop & ~is_loop & txp);
	rx_ready <= (~stop & ~is_loop & ~mon & ~full);
end

// set local MAC address
reg [47:0] mac;

always @(posedge clk) begin
	if (mac_wr && mac_strobe) begin
		mac <= { mac[39:0], mac_byte };
	end
end

reg [7:0] reg_do;
reg [7:0] rom_do;

reg [15:0] rx_buffer_do;
reg [15:0] tx_buffer_do;

// prom read
always @(*) begin
	case (rsar[3:1])
		3'h00: rom_do = mac[47:40];
		3'h01: rom_do = mac[39:32];
		3'h02: rom_do = mac[31:24];
		3'h03: rom_do = mac[23:16];
		3'h04: rom_do = mac[15:8];
		3'h05: rom_do = mac[7:0];
		3'h07: rom_do = 8'h57;
		default: rom_do = 8'h00;
	endcase
end

// register read
always @(*) begin
	reg_do = 8'd0;
	// page 0
	if (ps == 0) begin
		case (addr)
			5'h03: reg_do = bnry;
			5'h04: reg_do = tsr;
			5'h07: reg_do = isr;
			5'h08: reg_do = { rsar[7:1], 1'b0 };
			5'h09: reg_do = rsar[15:8];
			5'h0a: reg_do = 8'h50; // id0
			5'h0b: reg_do = 8'h70; // id1
			5'h0c: reg_do = rsr;
			default: ;
		endcase
	end
	// page 1
	if (ps == 1) begin
		case (addr)
			5'h01: reg_do = mac[47:40];
			5'h02: reg_do = mac[39:32];
			5'h03: reg_do = mac[31:24];
			5'h04: reg_do = mac[23:16];
			5'h05: reg_do = mac[15:8];
			5'h06: reg_do = mac[7:0];
			5'h07: reg_do = curr;
			default: ;
		endcase
	end
	// page 3
	if (ps == 3) begin
		case (addr)
			5'h03: reg_do = 8'h08; // jumper mode
			5'h04: reg_do = 8'h90; // irq 3, port 0x300
			default: ;
		endcase
	end
	// cr is available on all pages
	if (addr == 0) begin
		reg_do = cr;
	end
end

wire is_loop = (lb != 0);
wire is_prom = (rsar[15:8] == 0);

wire [15:0] dma_do = (is_prom) ? { rom_do, rom_do }
	: (is_loop) ? tx_buffer_do
	: rx_buffer_do;

// CPU read
always @(*) begin
	dout = 16'd0;
	if (rd) begin
		dout = (dma_port) ? dma_do : { reg_do, reg_do };
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
		end else if (rx_strobe) begin
			if (rx_addr[0] == 1'b0) begin
				lo_byte <= rx_byte;
			end else begin
				rx_buffer[rx_addr[11:1]] <= { rx_byte, lo_byte };
			end
			if (!rsr[2]) begin
				rx_length <= rx_length + 11'd1;
				// write past pstop boundary
				rx_addr <= rx_addr + 12'd1;
				// full page count
				if (rx_addr[7:0] == 8'hFF) begin
					clda <= next_clda;
				end
			end
			// check max frame limit
			if (rx_length == 1518) begin
				rsr[2] <= 1'b1; // FAE
			end
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
				rx_buffer[rx_addr[11:1]] <= { 5'h00, rx_length };
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

// local DMA frame transmitter
always @(posedge clk) begin
	tx_byte <= (tx_byte_sel) ? tx_buffer_do[15:8] : tx_buffer_do[7:0];
	if (rst) begin
		tsr <= 8'h00;
	end else begin
		if (tx_start) begin
			tsr[0] <= 1'b0;
			tx_addr <= { tpsr, 8'd0 };
		end else if (tx_strobe) begin
			tx_addr <= tx_addr + 12'd1;
		end else if (tx_done) begin
			tsr[0] <= 1'b1; // PTX
		end
	end
end

// remote DMA data reader
always @(posedge clk) begin
	tx_buffer_do <= tx_buffer[tx_r_addr];
	rx_buffer_do <= rx_buffer[rsar[11:1]];
end

// remote DMA data writer
always @(posedge clk) begin
	if (wr_ne) begin
		if (dma_port && !rbcr_is_0) begin
			tx_buffer[rsar[11:1]] <= din;
		end
	end
end

wire rbcr_is_0 = (rbcr[10:1] == 0);
wire rbcr_is_1 = (rbcr[10:1] == 1);

// CPU write
always @(posedge clk) begin
	if (rst || reset) begin
		cr  <= 8'h21; // ABORT, STP
		rcr <= 6'h20; // MON
		tcr <= 5'h02; // LB1
		isr <= 8'h80; // RST
		imr <= 7'h00;
		irq <= 1'b00;
	end else begin
		if (wr_ne) begin
			if (!dma_port) begin
				// cr is available on all pages
				if (addr == 0) begin
					{ cr[7:3], cr[1:0] } <= { din[7:3], din[1:0] };
					if (din[0]) begin
						// stop
						cr[2] <= 1'b0; // TXP
						isr[7] <= 1'b1; // RST
					end else begin
						// not stop
						if (stop) begin
							isr[7] <= 1'b0; // RST
						end
						if (din[2]) begin
							cr[2] <= 1'b1;  // TXP
						end
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
						5'h07: isr[6:0] <= isr[6:0] & ~din[6:0];
						5'h08: rsar[7:1]  <= din[7:1];
						5'h09: rsar[15:8] <= din[7:0];
						5'h0a: rbcr[7:0]  <= din[7:0];
						5'h0b: rbcr[10:1] <= { din[2:0], rbcr[7:1] } + { 9'd0, rbcr[0] };
						5'h0c: rcr <= din[5:0];
						5'h0d: tcr <= din[4:0];
						5'h0f: imr <= din[6:0];
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
				rbcr[10:1] <= rbcr[10:1] - 10'd1;
				if (rbcr_is_1) begin
					isr[6] <= 1'b1; // RDC
				end
			end
		end
		// incoming frame received
		if (rx_done) begin
			curr <= clda;
			if (!stop) begin
				isr[0] <= 1'b1; // PRX
			end
		end
		// outgoing frame transmitted
		if (tx_done) begin
			cr[2] <= 1'b0; // TXP
			tbcr <= 11'd0;
			if (!stop) begin
				isr[1] <= 1'b1; // PTX
			end
		end
		// interrupt line
		irq <= |(isr[6:0] & imr);
	end
end

endmodule
