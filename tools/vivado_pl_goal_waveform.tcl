# Vivado report helper: run PL goal-compute RTL simulation and open waveform.
# Usage in Vivado Tcl Console:
#   source C:/Users/hansung/examples/ultra_yubin_v1/tools/vivado_pl_goal_waveform.tcl

if {[catch {current_project} proj_name]} {
    error "Open ultra_yubin_v1.xpr first, then source this script."
}

set script_path [file normalize [info script]]
set repo_dir [file dirname [file dirname $script_path]]
set proj_dir [get_property DIRECTORY [current_project]]

set rtl_candidates [list \
    [file join $repo_dir hardware pl_goal_compute rtl pl_goal_compute_axi.v] \
    [file join $proj_dir hardware pl_goal_compute rtl pl_goal_compute_axi.v] \
]
set tb_candidates [list \
    [file join $repo_dir hardware pl_goal_compute tb pl_goal_compute_axi_tb.v] \
    [file join $proj_dir hardware pl_goal_compute tb pl_goal_compute_axi_tb.v] \
]

set rtl_file ""
foreach f $rtl_candidates {
    if {[file exists $f]} {
        set rtl_file [file normalize $f]
        break
    }
}

set tb_file ""
foreach f $tb_candidates {
    if {[file exists $f]} {
        set tb_file [file normalize $f]
        break
    }
}

if {$rtl_file eq ""} {
    error "Cannot find hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v. Pull/copy the full repo first."
}
if {$tb_file eq ""} {
    error "Cannot find hardware/pl_goal_compute/tb/pl_goal_compute_axi_tb.v. Pull/copy the full repo first."
}

puts "RTL: $rtl_file"
puts "TB : $tb_file"

if {[catch {get_filesets sim_1}]} {
    create_fileset -simset sim_1
}

catch {add_files -norecurse -fileset sources_1 $rtl_file}
catch {add_files -norecurse -fileset sim_1 $tb_file}
set_property top pl_goal_compute_axi_tb [get_filesets sim_1]
update_compile_order -fileset sim_1

launch_simulation
restart

add_wave -divider "Clock / Reset"
add_wave /pl_goal_compute_axi_tb/clk
add_wave /pl_goal_compute_axi_tb/rstn

add_wave -divider "AXI Write"
add_wave -radix hex /pl_goal_compute_axi_tb/awaddr
add_wave /pl_goal_compute_axi_tb/awvalid
add_wave /pl_goal_compute_axi_tb/awready
add_wave -radix unsigned /pl_goal_compute_axi_tb/wdata
add_wave /pl_goal_compute_axi_tb/wvalid
add_wave /pl_goal_compute_axi_tb/wready
add_wave /pl_goal_compute_axi_tb/bvalid
add_wave /pl_goal_compute_axi_tb/bready

add_wave -divider "AXI Read"
add_wave -radix hex /pl_goal_compute_axi_tb/araddr
add_wave /pl_goal_compute_axi_tb/arvalid
add_wave /pl_goal_compute_axi_tb/arready
add_wave -radix unsigned /pl_goal_compute_axi_tb/rdata
add_wave /pl_goal_compute_axi_tb/rvalid
add_wave /pl_goal_compute_axi_tb/rready

add_wave -divider "PL Track Inputs"
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/track_cx
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/track_cy
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/track_fw
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/track_fh
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/track_conf
add_wave /pl_goal_compute_axi_tb/dut/track_valid

add_wave -divider "PL Computed Goals"
add_wave -radix signed /pl_goal_compute_axi_tb/dut/track_error_x
add_wave -radix signed /pl_goal_compute_axi_tb/dut/track_error_y
add_wave -radix signed /pl_goal_compute_axi_tb/dut/track_pan_step
add_wave -radix signed /pl_goal_compute_axi_tb/dut/track_tilt_step
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/pan_goal
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/tilt_goal
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/last_pan
add_wave -radix unsigned /pl_goal_compute_axi_tb/dut/last_tilt

run all
wave zoom full

puts "Done. Check Tcl Console for TB_PASS and capture the waveform window."
