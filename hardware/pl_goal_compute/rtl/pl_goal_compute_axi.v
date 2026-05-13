`timescale 1ns / 1ps

module pl_goal_compute_axi #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
) (
    input  wire                              s_axi_aclk,
    input  wire                              s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire [2:0]                        s_axi_awprot,
    input  wire                              s_axi_awvalid,
    output reg                               s_axi_awready,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output reg                               s_axi_wready,

    output reg [1:0]                         s_axi_bresp,
    output reg                               s_axi_bvalid,
    input  wire                              s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire [2:0]                        s_axi_arprot,
    input  wire                              s_axi_arvalid,
    output reg                               s_axi_arready,

    output reg [C_S_AXI_DATA_WIDTH-1:0]      s_axi_rdata,
    output reg [1:0]                         s_axi_rresp,
    output reg                               s_axi_rvalid,
    input  wire                              s_axi_rready
);

    localparam [5:0] ADDR_CTRL        = 6'h00;
    localparam [5:0] ADDR_PAN_GOAL    = 6'h04;
    localparam [5:0] ADDR_TILT_GOAL   = 6'h08;
    localparam [5:0] ADDR_IDS         = 6'h0c;
    localparam [5:0] ADDR_STATUS      = 6'h10;
    localparam [5:0] ADDR_LAST_PAN    = 6'h14;
    localparam [5:0] ADDR_LAST_TILT   = 6'h18;
    localparam [5:0] ADDR_TRACK_XY    = 6'h34;
    localparam [5:0] ADDR_TRACK_FRAME = 6'h38;
    localparam [5:0] ADDR_TRACK_CMD   = 6'h3c;

    localparam [31:0] GOAL_MIN = 32'd0;
    localparam [31:0] GOAL_MAX = 32'd4095;

    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_q;
    reg [C_S_AXI_DATA_WIDTH-1:0] wdata_q;
    reg [(C_S_AXI_DATA_WIDTH/8)-1:0] wstrb_q;
    reg aw_valid_q;
    reg w_valid_q;

    reg [31:0] pan_goal;
    reg [31:0] tilt_goal;
    reg [7:0] pan_id;
    reg [7:0] tilt_id;
    reg done_toggle;
    reg track_toggle;
    reg audio_toggle;
    reg [31:0] compute_count;
    reg [31:0] last_pan;
    reg [31:0] last_tilt;

    reg [15:0] track_cx;
    reg [15:0] track_cy;
    reg [15:0] track_fw;
    reg [15:0] track_fh;
    reg [15:0] track_conf;
    reg signed [15:0] track_pan_gain;
    reg signed [15:0] track_tilt_gain;

    wire aw_take = s_axi_awready && s_axi_awvalid;
    wire w_take = s_axi_wready && s_axi_wvalid;
    wire write_fire = aw_valid_q && w_valid_q && !s_axi_bvalid;

    wire signed [31:0] track_error_x;
    wire signed [31:0] track_error_y;
    wire signed [47:0] track_pan_delta;
    wire signed [47:0] track_tilt_delta;

    assign track_error_x = $signed({16'h0, track_cx}) - $signed({16'h0, (track_fw >> 1)});
    assign track_error_y = $signed({16'h0, track_cy}) - $signed({16'h0, (track_fh >> 1)});
    assign track_pan_delta = (track_error_x * track_pan_gain) >>> 4;
    assign track_tilt_delta = (track_error_y * track_tilt_gain) >>> 4;

    function [31:0] apply_wstrb;
        input [31:0] old_data;
        input [31:0] new_data;
        input [3:0] strb;
        integer i;
        begin
            apply_wstrb = old_data;
            for (i = 0; i < 4; i = i + 1) begin
                if (strb[i]) begin
                    apply_wstrb[i*8 +: 8] = new_data[i*8 +: 8];
                end
            end
        end
    endfunction

    function [31:0] clamp_goal;
        input signed [63:0] value;
        begin
            if (value < $signed({32'h0, GOAL_MIN})) begin
                clamp_goal = GOAL_MIN;
            end else if (value > $signed({32'h0, GOAL_MAX})) begin
                clamp_goal = GOAL_MAX;
            end else begin
                clamp_goal = value[31:0];
            end
        end
    endfunction

    function [31:0] read_reg;
        input [C_S_AXI_ADDR_WIDTH-1:0] addr;
        begin
            case (addr[5:0] & 6'h3c)
                ADDR_CTRL:        read_reg = {16'h0, done_toggle, 6'h0, audio_toggle, track_toggle, 7'h0};
                ADDR_PAN_GOAL:    read_reg = pan_goal;
                ADDR_TILT_GOAL:   read_reg = tilt_goal;
                ADDR_IDS:         read_reg = {16'h0, tilt_id, pan_id};
                ADDR_STATUS:      read_reg = compute_count;
                ADDR_LAST_PAN:    read_reg = last_pan;
                ADDR_LAST_TILT:   read_reg = last_tilt;
                ADDR_TRACK_XY:    read_reg = {track_cy, track_cx};
                ADDR_TRACK_FRAME: read_reg = {track_fh, track_fw};
                ADDR_TRACK_CMD:   read_reg = {7'h0, track_conf, done_toggle, 6'h0, audio_toggle, track_toggle};
                default:          read_reg = 32'h0;
            endcase
        end
    endfunction

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bresp <= 2'b00;
            s_axi_bvalid <= 1'b0;
            awaddr_q <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            wdata_q <= {C_S_AXI_DATA_WIDTH{1'b0}};
            wstrb_q <= {(C_S_AXI_DATA_WIDTH/8){1'b0}};
            aw_valid_q <= 1'b0;
            w_valid_q <= 1'b0;
        end else begin
            s_axi_awready <= !aw_valid_q && !s_axi_bvalid;
            s_axi_wready <= !w_valid_q && !s_axi_bvalid;

            if (aw_take) begin
                awaddr_q <= s_axi_awaddr;
                aw_valid_q <= 1'b1;
            end
            if (w_take) begin
                wdata_q <= s_axi_wdata;
                wstrb_q <= s_axi_wstrb;
                w_valid_q <= 1'b1;
            end

            if (write_fire) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;
                aw_valid_q <= 1'b0;
                w_valid_q <= 1'b0;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rdata <= 32'h0;
            s_axi_rresp <= 2'b00;
            s_axi_rvalid <= 1'b0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rdata <= read_reg(s_axi_araddr);
                s_axi_rresp <= 2'b00;
                s_axi_rvalid <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            pan_goal <= 32'd2048;
            tilt_goal <= 32'd2772;
            pan_id <= 8'd1;
            tilt_id <= 8'd2;
            done_toggle <= 1'b0;
            track_toggle <= 1'b0;
            audio_toggle <= 1'b0;
            compute_count <= 32'h0;
            last_pan <= 32'd2048;
            last_tilt <= 32'd2772;
            track_cx <= 16'd640;
            track_cy <= 16'd360;
            track_fw <= 16'd1280;
            track_fh <= 16'd720;
            track_conf <= 16'h0;
            track_pan_gain <= 16'sd1;
            track_tilt_gain <= -16'sd1;
        end else if (write_fire) begin
            case (awaddr_q[5:0] & 6'h3c)
                ADDR_CTRL: begin
                    if (wdata_q[0]) begin
                        last_pan <= pan_goal;
                        last_tilt <= tilt_goal;
                        compute_count <= compute_count + 1'b1;
                        done_toggle <= ~done_toggle;
                    end
                end
                ADDR_PAN_GOAL: begin
                    pan_goal <= apply_wstrb(pan_goal, wdata_q, wstrb_q);
                end
                ADDR_TILT_GOAL: begin
                    tilt_goal <= apply_wstrb(tilt_goal, wdata_q, wstrb_q);
                end
                ADDR_IDS: begin
                    if (wstrb_q[0]) pan_id <= wdata_q[7:0];
                    if (wstrb_q[1]) tilt_id <= wdata_q[15:8];
                end
                ADDR_TRACK_XY: begin
                    if (wstrb_q[0]) track_cx[7:0] <= wdata_q[7:0];
                    if (wstrb_q[1]) track_cx[15:8] <= wdata_q[15:8];
                    if (wstrb_q[2]) track_cy[7:0] <= wdata_q[23:16];
                    if (wstrb_q[3]) track_cy[15:8] <= wdata_q[31:24];
                end
                ADDR_TRACK_FRAME: begin
                    if (wstrb_q[0]) track_fw[7:0] <= wdata_q[7:0];
                    if (wstrb_q[1]) track_fw[15:8] <= wdata_q[15:8];
                    if (wstrb_q[2]) track_fh[7:0] <= wdata_q[23:16];
                    if (wstrb_q[3]) track_fh[15:8] <= wdata_q[31:24];
                end
                ADDR_TRACK_CMD: begin
                    track_conf <= wdata_q[23:8];
                    if (wdata_q[0] && wdata_q[1] && track_fw != 16'h0 && track_fh != 16'h0) begin
                        pan_goal <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_delta));
                        tilt_goal <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_delta));
                        last_pan <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_delta));
                        last_tilt <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_delta));
                        compute_count <= compute_count + 1'b1;
                        done_toggle <= ~done_toggle;
                        track_toggle <= ~track_toggle;
                    end
                end
                default: begin
                end
            endcase
        end
    end

    wire unused_inputs = &{1'b0, s_axi_awprot, s_axi_arprot};

endmodule
