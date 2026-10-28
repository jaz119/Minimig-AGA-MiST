//
// X-Surf Ethernet card implementation for Minimig-MIST
// Copyright (c) 2026 Eugene Azarov <rusjaz@gmail.com>
//

module xsurf
(
    input             clk,        // bus clock
    input             rst,        // system reset
    input             sel,        // card select
    input             cpu_rd,     // cpu read enable
    input             cpu_hwr,    // high byte write enable
    input             cpu_lwr,    // low byte writes enable
    input      [15:1] addr,       // address bus (word aligned)
    input      [15:0] din,        // data bus
    output reg [15:0] dout,       // data bus
    output            int2,       // interrupt request

    // ethernet status word to be read by i/o controller
    output     [31:0] status,

    // interface to allow MAC address being set by i/o controller
    input             mac_begin,  // rising edge before new mac is sent
    input             mac_strobe, // rising edge before each mac byte
    input       [7:0] mac_byte,   // mac address byte

    // interface to allow the i/o controller to read frames from the tx buffer
    input             tx_begin,   // rising edge before new tx byte stream is sent
    input             tx_strobe,  // rising edge before each tx byte
    output      [7:0] tx_byte,    // byte from transmit buffer 

    // interface to allow the i/o controller to write frames to the rx buffer
    input             rx_begin,   // rising edge before new rx byte stream is sent
    input             rx_strobe,  // rising edge before each rx byte
    input       [7:0] rx_byte     // byte to be written to rx buffer 
);
    localparam ETH_BASE = 16'h8600;
    wire sel_eth = sel && (addr[15:6] == ETH_BASE[15:6]);

    wire rd = sel_eth && cpu_rd;
    wire wr = sel_eth && (cpu_hwr || cpu_lwr);

    wire [15:0] eth_dout;

    always @(*) begin
        dout = 16'd0;
        if (cpu_rd) begin
            if (sel_eth) begin
                // ethernet data
                dout = eth_dout;
            end else if (sel && (addr[15:1] == 15'h0020)) begin
                // reg 0x40: status of ethernet irq
                dout = { int2, 1'b0, int2, 13'd0 };
            end
        end
    end

    // all transactions are 16-bit only
    rtl8019 rtl8019_isa
    (
        .clk(clk),
        .rst(rst),
        .rd(rd),
        .wr(wr),

        .addr(addr[5:1]),
        .din(din),
        .dout(eth_dout),

        .int(int2),
        .status(status),

        .mac_begin(mac_begin),
        .mac_strobe(mac_strobe),
        .mac_byte(mac_byte),

        .tx_begin(tx_begin),
        .tx_strobe(tx_strobe),
        .tx_byte(tx_byte),

        .rx_begin(rx_begin),
        .rx_strobe(rx_strobe),
        .rx_byte(rx_byte)
    );

endmodule
