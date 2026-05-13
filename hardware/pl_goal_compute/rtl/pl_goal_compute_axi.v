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

    localparam integer ADDR_CTRL        = 6'h00;
    localparam integer ADDR_PAN_GOAL    = 6'h04;
    localparam integer ADDR_TILT_GOAL   = 6'h08;
    localparam integer ADDR_IDS         = 6'h0c;
    localparam integer ADDR_STATUS      = 6'h10;
    localparam integer ADDR_LAST_PAN    = 6'h14;
    localparam integer ADDR_LAST_TILT   = 6'h18;
    localparam integer ADDR_TRACK_XY    = 6'h34;
    localparam integer ADDR_TRACK_FRAME = 6'h38;
    localparam integer ADDR_TRACK_CMD   = 6'h3c;

    localparam [31:0] GOAL_MIN = 32'd0;
    localparam [31:0] GOAL_MAX = 32'd4095;
    localparam signed [31:0] PAN_CENTER = 32'sd2048;

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
    reg signed [15:0] audio_angle_deg;

    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_q;
    reg [C_S_AXI_DATA_WIDTH-1:0] wdata_q;
    reg [(C_S_AXI_DATA_WIDTH/8)-1:0] wstrb_q;
    reg aw_seen;
    reg w_seen;

    wire aw_hs = s_axi_awready && s_axi_awvalid;
    wire w_hs = s_axi_wready && s_axi_wvalid;
    wire write_fire = !s_axi_bvalid && (aw_seen || aw_hs) && (w_seen || w_hs);
    wire [C_S_AXI_ADDR_WIDTH-1:0] write_addr = aw_hs ? s_axi_awaddr : awaddr_q;
    wire [C_S_AXI_DATA_WIDTH-1:0] write_data = w_hs ? s_axi_wdata : wdata_q;
    wire [(C_S_AXI_DATA_WIDTH/8)-1:0] write_strb = w_hs ? s_axi_wstrb : wstrb_q;
    wire read_fire = s_axi_arready && s_axi_arvalid;

    wire signed [31:0] track_error_x;
    wire signed [31:0] track_error_y;
    wire signed [47:0] track_pan_delta;
    wire signed [47:0] track_tilt_delta;
    wire signed [47:0] audio_pan_delta;
    wire signed [47:0] track_audio_pan_delta;

    assign track_error_x = $signed({16'h0, track_cx}) - $signed({16'h0, (track_fw >> 1)});
    assign track_error_y = $signed({16'h0, track_cy}) - $signed({16'h0, (track_fh >> 1)});
    assign track_pan_delta = (track_error_x * track_pan_gain) >>> 4;
    assign track_tilt_delta = (track_error_y * track_tilt_gain) >>> 4;
    assign audio_pan_delta = $signed(audio_angle_deg) * 48'sd11;
    assign track_audio_pan_delta = $signed(track_cx) * 48'sd11;

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

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bresp <= 2'b00;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rdata <= 32'h0;
            s_axi_rresp <= 2'b00;
            s_axi_rvalid <= 1'b0;
            awaddr_q <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            wdata_q <= {C_S_AXI_DATA_WIDTH{1'b0}};
            wstrb_q <= {(C_S_AXI_DATA_WIDTH/8){1'b0}};
            aw_seen <= 1'b0;
            w_seen <= 1'b0;
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
            audio_angle_deg <= 16'sd0;
        end else begin
            s_axi_awready <= !aw_seen && !s_axi_bvalid;
            s_axi_wready <= !w_seen && !s_axi_bvalid;

            if (aw_hs && !write_fire) begin
                awaddr_q <= s_axi_awaddr;
                aw_seen <= 1'b1;
            end
            if (w_hs && !write_fire) begin
                wdata_q <= s_axi_wdata;
                wstrb_q <= s_axi_wstrb;
                w_seen <= 1'b1;
            end

            if (write_fire) begin
                case (write_addr[5:0] & 6'h3c)
                    ADDR_CTRL: begin
                        if (write_data[0]) begin
                            last_pan <= pan_goal;
                            last_tilt <= tilt_goal;
                            compute_count <= compute_count + 1'b1;
                            done_toggle <= ~done_toggle;
                        end
                    end
                    ADDR_PAN_GOAL: begin
                        pan_goal <= apply_wstrb(pan_goal, write_data, write_strb);
                    end
                    ADDR_TILT_GOAL: begin
                        tilt_goal <= apply_wstrb(tilt_goal, write_data, write_strb);
                    end
                    ADDR_IDS: begin
                        if (write_strb[0]) pan_id <= write_data[7:0];
                        if (write_strb[1]) tilt_id <= write_data[15:8];
                    end
                    ADDR_TRACK_XY: begin
                        if (write_strb[0]) track_cx[7:0] <= write_data[7:0];
                        if (write_strb[1]) track_cx[15:8] <= write_data[15:8];
                        if (write_strb[2]) track_cy[7:0] <= write_data[23:16];
                        if (write_strb[3]) track_cy[15:8] <= write_data[31:24];
                    end
                    ADDR_TRACK_FRAME: begin
                        if (write_strb[0]) track_fw[7:0] <= write_data[7:0];
                        if (write_strb[1]) track_fw[15:8] <= write_data[15:8];
                        if (write_strb[2]) track_fh[7:0] <= write_data[23:16];
                        if (write_strb[3]) track_fh[15:8] <= write_data[31:24];
                    end
                    ADDR_TRACK_CMD: begin
                        track_conf <= write_data[23:8];
                        case (write_data[2:1])
                            2'd1: begin
                                if (write_data[0] && track_fw != 16'h0 && track_fh != 16'h0) begin
                                    pan_goal <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_delta));
                                    tilt_goal <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_delta));
                                    last_pan <= clamp_goal($signed({32'h0, pan_goal}) + $signed(track_pan_delta));
                                    last_tilt <= clamp_goal($signed({32'h0, tilt_goal}) + $signed(track_tilt_delta));
                                    compute_count <= compute_count + 1'b1;
                                    done_toggle <= ~done_toggle;
                                    track_toggle <= ~track_toggle;
                                end
                            end
                            2'd2: begin
                                audio_angle_deg <= track_cx;
                                if (write_data[0]) begin
                                    pan_goal <= clamp_goal($signed(PAN_CENTER) + $signed(track_audio_pan_delta));
                                    last_pan <= clamp_goal($signed(PAN_CENTER) + $signed(track_audio_pan_delta));
                                    last_tilt <= tilt_goal;
                                    compute_count <= compute_count + 1'b1;
                                    done_toggle <= ~done_toggle;
                                    audio_toggle <= ~audio_toggle;
                                end
                            end
                            default: begin
                            end
                        endcase
                    end
                    default: begin
                    end
                endcase
                aw_seen <= 1'b0;
                w_seen <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            s_axi_arready <= !s_axi_rvalid;

            if (read_fire) begin
                case (s_axi_araddr[5:0] & 6'h3c)
                    ADDR_CTRL:        s_axi_rdata <= {16'h0, done_toggle, 6'h0, audio_toggle, track_toggle, 7'h0};
                    ADDR_PAN_GOAL:    s_axi_rdata <= pan_goal;
                    ADDR_TILT_GOAL:   s_axi_rdata <= tilt_goal;
                    ADDR_IDS:         s_axi_rdata <= {16'h0, tilt_id, pan_id};
                    ADDR_STATUS:      s_axi_rdata <= compute_count;
                    ADDR_LAST_PAN:    s_axi_rdata <= last_pan;
                    ADDR_LAST_TILT:   s_axi_rdata <= last_tilt;
                    ADDR_TRACK_XY:    s_axi_rdata <= {track_cy, track_cx};
                    ADDR_TRACK_FRAME: s_axi_rdata <= {track_fh, track_fw};
                    ADDR_TRACK_CMD:   s_axi_rdata <= {8'h0, track_conf, done_toggle, 6'h0, audio_toggle, track_toggle};
                    default:          s_axi_rdata <= 32'h0;
                endcase
                s_axi_rvalid <= 1'b1;
                s_axi_rresp <= 2'b00;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule
