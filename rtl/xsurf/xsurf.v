//
// X-Surf Ethernet card implementation for Minimig-MIST
// Copyright (c) 2026 Eugene Azarov <rusjaz@gmail.com>
//

module xsurf
(
    input             clk,
    input             rst,
    input             sel,                  // card select
    input             cpu_rd,               // cpu read enable
    input             cpu_hwr,              // high byte write enable
    input             cpu_lwr,              // low byte writes enable
    input      [15:1] addr,                 // address bus (word aligned)
    input      [15:0] din,
    output reg [15:0] dout,
    output            int2,                 // interrupt request
    output     [31:0] eth_status,           // ethernet nic status
    input             eth_mac_begin,
    input             eth_mac_strobe,
    input       [7:0] eth_mac_byte,
    input             eth_tx_read_begin,
    input             eth_tx_read_strobe,
    output      [7:0] eth_tx_read_byte,
    input             eth_rx_write_begin,
    input             eth_rx_write_strobe,
    input       [7:0] eth_rx_write_byte
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
    rtl8019 rtl8019_eth
    (
        .clk(clk),
        .rst(rst),
        .rd(rd),
        .wr(wr),

        .addr(addr[5:1]),
        .din(din),
        .dout(eth_dout),

        .irq(int2),
        .status(eth_status),

        .mac_begin(eth_mac_begin),
        .mac_strobe(eth_mac_strobe),
        .mac_byte(eth_mac_byte),

        .tx_begin(eth_tx_read_begin),
        .tx_strobe(eth_tx_read_strobe),
        .tx_byte(eth_tx_read_byte),

        .rx_begin(eth_rx_write_begin),
        .rx_strobe(eth_rx_write_strobe),
        .rx_byte(eth_rx_write_byte)
    );

endmodule
