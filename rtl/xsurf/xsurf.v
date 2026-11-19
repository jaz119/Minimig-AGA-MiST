//
// X-Surf Ethernet card implementation for Minimig
// Copyright (c) 2026 Eugene Azarov <rusjaz@gmail.com>
//
// SPDX-License-Identifier: GPL-2.0-or-later
//

module xsurf
(
    input             clk,
    input             clk7_en,
    input             rst,
    input             sel,
    input             cpu_rd,
    input             cpu_hwr,
    input             cpu_lwr,
    input      [15:1] addr,
    input      [15:0] din,
    output reg [15:0] dout,
    output reg        int2,

    output [31:0] eth_status,
    input         eth_mac_wr,
    input         eth_mac_strobe,
    input   [7:0] eth_mac_byte,
    input         eth_tx_rd,
    input         eth_tx_strobe,
    output  [7:0] eth_tx_byte,
    input         eth_rx_wr,
    input         eth_rx_strobe,
    input   [7:0] eth_rx_byte
);

localparam ETH_BASE = 16'h8600; // port 0x300

wire sel_eth = (addr[15:6] == ETH_BASE[15:6]);
wire sel_40h = (addr[7:1] == 7'h20) && !addr[11] && !addr[15];

wire eth_irq;
wire [15:0] eth_dout;
wire  [7:0] status = { int2, 1'b0, int2, 5'b0 };

always @(*) begin
    dout = 16'd0;
    if (sel && cpu_rd) begin
        if (sel_eth) begin
            // ethernet data
            dout = eth_dout;
        end else if (sel_40h) begin
            // reg 0x40: status of ethernet irq
            dout = { status, status };
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        int2 <= 1'b0;
    end else if (clk7_en) begin
        int2 <= eth_irq;
    end
end

wire rd = sel && sel_eth & cpu_rd;
wire wr = sel && sel_eth & (cpu_hwr | cpu_lwr);

rtl8019 rtl8019_eth
(
    .clk(clk),
    .rst(rst),
    .rd(rd),
    .wr(wr),

    .addr(addr[5:1]),
    .din({ din[7:0], din[15:8] }),
    .dout({ eth_dout[7:0], eth_dout[15:8] }),

    .irq(eth_irq),
    .status(eth_status),

    .mac_wr(eth_mac_wr),
    .mac_strobe(eth_mac_strobe),
    .mac_byte(eth_mac_byte),

    .tx_rd(eth_tx_rd),
    .tx_strobe(eth_tx_strobe),
    .tx_byte(eth_tx_byte),

    .rx_wr(eth_rx_wr),
    .rx_strobe(eth_rx_strobe),
    .rx_byte(eth_rx_byte)
);

endmodule
