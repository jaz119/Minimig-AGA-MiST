module user_io( 

	input        clk_sys,
	input        SPI_CLK,
	input        SPI_SS_IO,
	output       reg SPI_MISO,
	input        SPI_MOSI,
	input  [7:0] CORE_TYPE,

	output [15:0] JOY0,
	output [15:0] JOY1,
	output [15:0] JOY2,
	output [15:0] JOY3,
	output reg   [15:0] JOY_ANA0,
	output reg   [15:0] JOY_ANA1,

	output [2:0] MOUSE0_BUTTONS,
	output [2:0] MOUSE1_BUTTONS,
	output       KBD_MOUSE_STROBE,
	output       KMS_LEVEL,
	output [1:0] KBD_MOUSE_TYPE,
	output [7:0] KBD_MOUSE_DATA,
	output       MOUSE_IDX,

	output [1:0] BUTTONS,
	output [1:0] SWITCHES,
	output [3:0] CONF,
	output [7:0] STATUS,

	output [63:0] RTC,

	// ethernet interface
	input [31:0]  eth_status,

	output reg          eth_mac_begin,
	output reg          eth_mac_strobe,
	output reg    [7:0] eth_mac_byte,

	output reg          eth_tx_read_begin,
	output reg          eth_tx_read_strobe,
	input         [7:0] eth_tx_read_byte,

	output reg          eth_rx_write_begin,
	output reg          eth_rx_write_strobe,
	output reg    [7:0] eth_rx_write_byte,

	// i2c bridge
	output reg          i2c_start,
	output reg          i2c_read,
	output reg    [6:0] i2c_addr,
	output reg    [7:0] i2c_subaddr,
	output reg    [7:0] i2c_dout,
	input         [7:0] i2c_din,
	input               i2c_ack,
	input               i2c_end,
	input               hdmi_hiclk // HDMI PCLK > 80 MHz
);

wire spi_sck = SPI_CLK;

reg [6:0]     sbuf;
reg [7:0]     cmd;
reg [3:0]     bit_cnt;    // counts bits 0-7 8-15 8-15 ...
reg [3:0]     byte_cnt;   // counts bytes
reg [7:0]     but_sw;
reg [7:0]     status;

reg           kbd_mouse_strobe;
reg           kbd_mouse_strobe_level;
reg [1:0]     kbd_mouse_type;
reg [7:0]     kbd_mouse_data;
reg           mouse_idx;
reg [2:0]     mouse0_buttons;
reg [2:0]     mouse1_buttons;
reg [31:0]    joystick_0;
reg [31:0]    joystick_1;
reg [31:0]    joystick_2;
reg [31:0]    joystick_3;
reg [31:0]    joystick_4;
reg  [2:0]    stick_idx;
reg [63:0]    rtc;

assign JOY0 = joystick_0[15:0];
assign JOY1 = joystick_1[15:0];
assign JOY2 = joystick_2[15:0];
assign JOY3 = joystick_3[15:0];
assign RTC  = rtc;

assign MOUSE_IDX = mouse_idx; // select mouse
assign KBD_MOUSE_DATA = kbd_mouse_data; // 8 bit movement data
assign KBD_MOUSE_TYPE = kbd_mouse_type; // 0=mouse x,1=mouse y, 2=keycode, 3=OSD kbd
assign KBD_MOUSE_STROBE = kbd_mouse_strobe; // strobe, data valid on rising edge
assign KMS_LEVEL = kbd_mouse_strobe_level; // level change of kbd_mouse_strobe
assign MOUSE0_BUTTONS = mouse0_buttons; // state of the two mouse buttons for first mouse
assign MOUSE1_BUTTONS = mouse1_buttons; // state of the two mouse buttons for second mouse

assign BUTTONS  = but_sw[1:0];
assign SWITCHES = but_sw[3:2];
assign CONF     = but_sw[7:4];
assign STATUS   = status;

// SPI bit and byte counters
always @(posedge spi_sck or posedge SPI_SS_IO) begin
	if (SPI_SS_IO == 1) begin
		bit_cnt <= 4'd0;
		byte_cnt <= 4'd0;
	end else begin
		// count 0-7 8-15 8-15 ...
		if (bit_cnt != 4'd15)
			bit_cnt <= bit_cnt + 4'd1;
		else begin
			bit_cnt <= 4'd8;
			if (~&byte_cnt)
				byte_cnt <= byte_cnt + 4'd1;
		end
	end
end

reg [7:0] spi_byte_out;

always @(negedge spi_sck or posedge SPI_SS_IO) begin : spi_byteout
	if(SPI_SS_IO == 1) begin
		SPI_MISO <= 1'bZ;
	end else begin
		SPI_MISO <= spi_byte_out[~bit_cnt[2:0]];
	end
end

always @(posedge spi_sck or posedge SPI_SS_IO) begin : spi_transmitter

	if(SPI_SS_IO == 1) begin
		spi_byte_out <= CORE_TYPE;
	end else begin
		// read the command byte to choose the response
		if(bit_cnt == 7) begin
			if(!byte_cnt) cmd <= {sbuf, SPI_MOSI};

			spi_byte_out <= 8'd0;
			case({(!byte_cnt) ? {sbuf, SPI_MOSI} : cmd})

			// ethernet status
			8'h0a: begin
				spi_byte_out <= eth_status[((byte_cnt-1)<<3) +:8];
			end

			// read ethernet tx buffer
			8'h0b: begin
				spi_byte_out <= eth_tx_read_byte;
			end

			// i2c
			8'h31:
				if (byte_cnt == 0) spi_byte_out <= {5'd0, hdmi_hiclk, i2c_ack, i2c_end};
				else spi_byte_out <= i2c_din;
			default: ;
			endcase
		end
	end
end

// SPI receiver IO -> FPGA

reg       spi_receiver_strobe_r = 0;
reg       spi_transfer_end_r = 1;
reg [7:0] spi_byte_in;

// Read at spi_sck clock domain, assemble bytes for transferring to clk_sys
always @(posedge spi_sck or posedge SPI_SS_IO) begin
	if (SPI_SS_IO == 1) begin
		spi_transfer_end_r <= 1;
		eth_tx_read_begin <= 1'b0;
		eth_tx_read_strobe <= 1'b0;

	end else begin
		spi_transfer_end_r <= 0;

		// command byte finished
		if (bit_cnt == 7) begin
			spi_byte_in <= { sbuf, SPI_MOSI };
			spi_receiver_strobe_r <= ~spi_receiver_strobe_r;

			if ({ sbuf, SPI_MOSI } == 8'h0b) begin
				eth_tx_read_begin <= 1'b1;
				eth_tx_read_strobe <= 1'b0;
			end
		end

		sbuf[6:1] <= sbuf[5:0];
		sbuf[0] <= SPI_MOSI;

		if (eth_tx_read_strobe && bit_cnt == 9) begin
			eth_tx_read_strobe <= 1'b0;
		end

		// payload byte finished
		if (bit_cnt == 15) begin
			spi_byte_in <= { sbuf, SPI_MOSI };
			spi_receiver_strobe_r <= ~spi_receiver_strobe_r;

			// give strobe after each eth byte read
			if (cmd == 8'h0b)
				eth_tx_read_strobe <= 1'b1;
		end
	end
end

// Process bytes from SPI at the clk_sys domain
always @(posedge clk_sys) begin

	reg       spi_receiver_strobe;
	reg       spi_transfer_end;
	reg       spi_receiver_strobeD;
	reg       spi_transfer_endD;
	reg [7:0] acmd;
	reg [7:0] abyte_cnt;   // counts bytes

	//synchronize between SPI and sys clock domains
	spi_receiver_strobeD <= spi_receiver_strobe_r;
	spi_receiver_strobe <= spi_receiver_strobeD;
	spi_transfer_endD	<= spi_transfer_end_r;
	spi_transfer_end	<= spi_transfer_endD;

	// strobe is set whenever a valid byte has been received
	kbd_mouse_strobe <= 0;

	i2c_start <= 0;
	eth_mac_begin <= 1'b0;
	eth_mac_strobe <= 1'b0;
	eth_rx_write_strobe <= 1'b0;

	if (spi_transfer_end) begin
		abyte_cnt <= 8'd0;
		eth_mac_begin <= 1'b0;
		eth_rx_write_begin <= 1'b0;
	end else if (spi_receiver_strobeD ^ spi_receiver_strobe) begin

		if(~&abyte_cnt) 
			abyte_cnt <= abyte_cnt + 8'd1;

		if(abyte_cnt == 0) begin
			acmd <= spi_byte_in;
			case (spi_byte_in)
				8'h70, 8'h71: begin
					kbd_mouse_type <= 2'b00;  // x axis
					mouse_idx <= spi_byte_in[0]; // mouse select
				end
				8'h05: kbd_mouse_type <= 2'b10;  // keyboard
		 		8'h06: kbd_mouse_type <= 2'b11;  // OSD keyboard
		 		8'h09: eth_mac_begin <= 1'b1;
		 		8'h0c: eth_rx_write_begin <= 1'b1;
			endcase
		end else begin
			case(acmd)
				// buttons and switches
				8'h01: but_sw <= spi_byte_in;
				8'h60: if (abyte_cnt < 5) joystick_0[((abyte_cnt-1)<<3) +:8] <= spi_byte_in;
				8'h61: if (abyte_cnt < 5) joystick_1[((abyte_cnt-1)<<3) +:8] <= spi_byte_in;
				8'h62: if (abyte_cnt < 5) joystick_2[((abyte_cnt-1)<<3) +:8] <= spi_byte_in;
				8'h63: if (abyte_cnt < 5) joystick_3[((abyte_cnt-1)<<3) +:8] <= spi_byte_in;
				8'h64: if (abyte_cnt < 5) joystick_4[((abyte_cnt-1)<<3) +:8] <= spi_byte_in;
				8'h70, 8'h71:
				begin
					if (abyte_cnt == 8'd1) begin
						kbd_mouse_data <= spi_byte_in;
						kbd_mouse_strobe <= 1;
						kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
					end else if (abyte_cnt == 8'd2) begin
						// second byte contains y movement data
						kbd_mouse_data <= spi_byte_in;
						kbd_mouse_strobe <= 1;
						kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
						kbd_mouse_type <= 2'b01;
					end else if (abyte_cnt == 8'd3) begin
						// third byte contains the buttons
						if (~mouse_idx)
							mouse0_buttons[2:0] <= spi_byte_in[2:0];
						else
							mouse1_buttons[2:0] <= spi_byte_in[2:0];
					end else begin
						// fourth byte contains wheel movement
						kbd_mouse_data <= spi_byte_in;
						kbd_mouse_strobe <= 1;
						kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
					end
				end
				8'h05, 8'h06:
				begin
					kbd_mouse_data <= spi_byte_in;
					kbd_mouse_strobe <= 1;
					kbd_mouse_strobe_level <= ~kbd_mouse_strobe_level;
				end
				// send mac address byte to ethernet controller
				8'h09: begin
					eth_mac_byte <= spi_byte_in;
					eth_mac_strobe <= 1'b1;
				end
				// send received byte to ethernet controller
				8'h0c: begin
					eth_rx_write_byte <= spi_byte_in;
					eth_rx_write_strobe <= 1'b1;
				end
				// joystick analog
				8'h1a: begin
					// first byte is joystick index
					if(abyte_cnt == 1)
						stick_idx <= spi_byte_in[2:0];
					else if(abyte_cnt == 2) begin
						// second byte is x axis
						if(stick_idx == 0)
							JOY_ANA0[15:8] <= {~spi_byte_in[7], spi_byte_in[6:0]};
						else if(stick_idx == 1)
							JOY_ANA1[15:8] <= {~spi_byte_in[7], spi_byte_in[6:0]};
					end else if(abyte_cnt == 3) begin
						// third byte is y axis
						if(stick_idx == 0)
							JOY_ANA0[7:0] <= {~spi_byte_in[7], spi_byte_in[6:0]};
						else if(stick_idx == 1)
							JOY_ANA1[7:0] <= {~spi_byte_in[7], spi_byte_in[6:0]};
					end
				end
				8'h22: if (abyte_cnt < 9) rtc[((abyte_cnt-1)<<3) +:8] <= spi_byte_in;
				8'h15: status <= spi_byte_in;
				// I2C bridge
				8'h30: if(abyte_cnt == 1) {i2c_addr, i2c_read} <= spi_byte_in;
				       else if (abyte_cnt == 2) i2c_subaddr <= spi_byte_in;
				       else if (abyte_cnt == 3) begin i2c_dout <= spi_byte_in; i2c_start <= 1; end
			endcase
		end
	end
end

endmodule
