# Vivado batch build for ultra_yubin PL goal compute project.
#
# Usage:
#   vivado -mode batch -source vivado_build_ultra_yubin.tcl -tclargs <project_root> <project_name> <pl_goal_compute_axi.v> [pl_goal_compute_axi_tb.v]

proc fail {msg} {
    puts stderr "BUILD_ERROR: $msg"
    exit 1
}

if {$argc < 3} {
    fail "usage: vivado_build_ultra_yubin.tcl <project_root> <project_name> <pl_goal_compute_axi.v>"
}

set project_root [lindex $argv 0]
set project_name [lindex $argv 1]
set rtl [lindex $argv 2]
set tb ""
if {$argc >= 4} {
    set tb [lindex $argv 3]
}
set part_name "xczu3eg-sbva484-1-i"
set xpr "$project_root/$project_name.xpr"
set board_part ""

if {![file exists $rtl]} { fail "RTL not found: $rtl" }
if {$tb ne "" && ![file exists $tb]} { fail "testbench not found: $tb" }
file mkdir $project_root

puts "===== OPEN / CREATE PROJECT ====="
set candidates {}
foreach pattern {*ultra96v2* *Ultra96v2* *ultra96* *Ultra96* *ULTRA96*} {
    foreach bp [get_board_parts -quiet $pattern] {
        if {[lsearch -exact $candidates $bp] < 0} {
            lappend candidates $bp
        }
    }
}
if {[llength $candidates] > 0} {
    set board_part [lindex $candidates 0]
    puts "BOARD_PART=$board_part"
} else {
    puts "Available board parts containing avnet:"
    foreach bp [get_board_parts -quiet *avnet*] { puts "  $bp" }
    fail "Ultra96 board_part not found. Install/select Ultra96-V2 board files before building this bitstream."
}

create_project -force $project_name $project_root -part $part_name
if {$board_part ne ""} {
    set_property board_part $board_part [current_project]
}
puts "PROJECT=[current_project]"
puts "DIR=[get_property DIRECTORY [current_project]]"
puts "PART=[get_property PART [current_project]]"
puts "BOARD=[get_property BOARD_PART [current_project]]"

puts "===== SOURCE CHECK ====="
if {[llength [get_files -quiet $rtl]] == 0} {
    add_files -fileset sources_1 $rtl
}
set fp [open $rtl r]
set txt [read $fp]
close $fp
if {[string first {module pl_goal_compute_axi} $txt] < 0} {
    fail "RTL does not contain pl_goal_compute_axi"
}
if {[string first {ADDR_TRACK_CMD} $txt] < 0} {
    fail "RTL does not contain ADDR_TRACK_CMD"
}
update_compile_order -fileset sources_1

if {$tb ne ""} {
    puts "===== RTL SIMULATION ====="
    if {[llength [get_files -quiet $tb]] == 0} {
        add_files -fileset sim_1 $tb
    }
    set_property top pl_goal_compute_axi_tb [get_filesets sim_1]
    set_property top_lib xil_defaultlib [get_filesets sim_1]
    update_compile_order -fileset sim_1
    set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]
    set sim_dir "$project_root/$project_name.sim/sim_1/behav/xsim"
    set sim_pass "$sim_dir/pl_goal_compute_axi_tb.pass"
    if {[file exists $sim_pass]} {
        file delete -force $sim_pass
    }
    launch_simulation -simset sim_1 -mode behavioral
    if {![file exists $sim_pass]} {
        close_sim
        fail "RTL simulation did not create pass sentinel: $sim_pass"
    }
    set fp [open $sim_pass r]
    set sim_txt [read $fp]
    close $fp
    if {[string first {TB_PASS: pl_goal_compute_axi AXI register and track path} $sim_txt] < 0} {
        close_sim
        fail "RTL simulation pass sentinel did not contain TB_PASS"
    }
    close_sim
    puts "RTL_SIM=PASS"
}

puts "===== CREATE / UPDATE BLOCK DESIGN ====="
create_bd_design design_1
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ultra_ps_e_0

if {$board_part ne ""} {
    catch {
        apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
            -config {apply_board_preset "1"} [get_bd_cells /zynq_ultra_ps_e_0]
    } board_preset_msg
    if {$board_preset_msg ne ""} {
        puts "WARN: board preset message: $board_preset_msg"
    }
}

catch {
    set_property -dict [list \
        CONFIG.PSU__USE__M_AXI_GP0 {1} \
        CONFIG.PSU__MAXIGP0__DATA_WIDTH {32} \
        CONFIG.PSU__FPGA_PL0_ENABLE {1} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
    ] [get_bd_cells /zynq_ultra_ps_e_0]
} ps_cfg_msg
if {$ps_cfg_msg ne ""} { puts "WARN: PS config best-effort message: $ps_cfg_msg" }

if {[llength [get_bd_intf_pins -quiet /zynq_ultra_ps_e_0/DDR]] != 0} {
    make_bd_intf_pins_external [get_bd_intf_pins /zynq_ultra_ps_e_0/DDR]
}
if {[llength [get_bd_intf_pins -quiet /zynq_ultra_ps_e_0/FIXED_IO]] != 0} {
    make_bd_intf_pins_external [get_bd_intf_pins /zynq_ultra_ps_e_0/FIXED_IO]
}
save_bd_design

if {[llength [get_bd_intf_pins -quiet /zynq_ultra_ps_e_0/M_AXI_HPM0_FPD]] == 0} {
    catch {
        set_property -dict [list \
            CONFIG.PSU__USE__M_AXI_GP0 {1} \
            CONFIG.PSU__MAXIGP0__DATA_WIDTH {32} \
            CONFIG.PSU__FPGA_PL0_ENABLE {1} \
            CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
        ] [get_bd_cells /zynq_ultra_ps_e_0]
    }
    save_bd_design
}

if {[llength [get_bd_cells -quiet /zynq_ultra_ps_e_0]] == 0} {
    fail "design_1.bd exists but /zynq_ultra_ps_e_0 is missing"
}
if {[llength [get_bd_intf_pins -quiet /zynq_ultra_ps_e_0/M_AXI_HPM0_FPD]] == 0} {
    fail "PS M_AXI_HPM0_FPD is missing. Enable AXI HPM0 FPD in Zynq MPSoC settings."
}
if {[llength [get_bd_pins -quiet /zynq_ultra_ps_e_0/pl_clk0]] == 0} {
    fail "PS pl_clk0 is missing. Enable PL Fabric Clock 0 at 100MHz."
}

set cell [get_bd_cells -quiet /pl_goal_compute_axi_pl]
if {[llength $cell] == 0} {
    puts "Creating /pl_goal_compute_axi_pl"
    set cell [create_bd_cell -type module -reference pl_goal_compute_axi pl_goal_compute_axi_pl]
} else {
    catch {update_module_reference $cell} update_msg
    if {$update_msg ne ""} { puts "WARN: update_module_reference message: $update_msg" }
}

set s_axi_nets [get_bd_nets -quiet -of_objects [get_bd_intf_pins /pl_goal_compute_axi_pl/s_axi]]
if {[llength $s_axi_nets] == 0} {
    puts "Connecting /pl_goal_compute_axi_pl/s_axi to PS M_AXI_HPM0_FPD"
    apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {
        Clk_master {/zynq_ultra_ps_e_0/pl_clk0 (100 MHz)}
        Clk_slave {/zynq_ultra_ps_e_0/pl_clk0 (100 MHz)}
        Clk_xbar {/zynq_ultra_ps_e_0/pl_clk0 (100 MHz)}
        Master {/zynq_ultra_ps_e_0/M_AXI_HPM0_FPD}
        Slave {/pl_goal_compute_axi_pl/s_axi}
        master_apm {0}
    } [get_bd_intf_pins /pl_goal_compute_axi_pl/s_axi]
}

assign_bd_address
foreach s [get_bd_addr_segs] {
    if {[string first {pl_goal_compute_axi_pl} $s] >= 0 || [string first {SEG_pl_goal_compute_axi_pl} $s] >= 0} {
        catch {set_property OFFSET 0x00A0000000 $s}
        catch {set_property RANGE 0x00001000 $s}
    }
    puts "$s OFFSET=[get_property OFFSET $s] RANGE=[get_property RANGE $s]"
}

foreach p [get_bd_pins -quiet -filter {TYPE == clk && DIR == I} /zynq_ultra_ps_e_0/*] {
    if {[llength [get_bd_nets -quiet -of_objects $p]] == 0} {
        catch {connect_bd_net [get_bd_pins /zynq_ultra_ps_e_0/pl_clk0] $p}
    }
}

puts "===== FORCE PL AXI RESET DEASSERTED ====="
set rst_const [get_bd_cells -quiet /pl_axi_resetn_const]
if {[llength $rst_const] == 0} {
    set rst_const [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 pl_axi_resetn_const]
}
set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {1}] $rst_const
foreach rst_pin {
    /pl_goal_compute_axi_pl/s_axi_aresetn
    /ps8_0_axi_periph/ARESETN
    /ps8_0_axi_periph/S00_ARESETN
    /ps8_0_axi_periph/M00_ARESETN
} {
    if {[llength [get_bd_pins -quiet $rst_pin]] != 0} {
        set rst_net [get_bd_nets -quiet -of_objects [get_bd_pins $rst_pin]]
        if {[llength $rst_net] != 0} {
            catch {disconnect_bd_net $rst_net [get_bd_pins $rst_pin]} disconnect_rst_msg
        }
        catch {connect_bd_net [get_bd_pins /pl_axi_resetn_const/dout] [get_bd_pins $rst_pin]} connect_rst_msg
        if {$connect_rst_msg ne ""} { puts "WARN: reset connect message for $rst_pin: $connect_rst_msg" }
    }
}

validate_bd_design
save_bd_design
set_property synth_checkpoint_mode None [get_files design_1.bd]
reset_target all [get_files design_1.bd]
generate_target all [get_files design_1.bd]

set wrapper_path [make_wrapper -fileset sources_1 -files [get_files -norecurse design_1.bd] -top]
add_files -norecurse -fileset sources_1 $wrapper_path
set_property top design_1_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "===== BUILD ====="
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

set synth_status [get_property STATUS [get_runs synth_1]]
set impl_status [get_property STATUS [get_runs impl_1]]
puts "SYNTH=$synth_status"
puts "IMPL=$impl_status"
if {[string first {Complete} $synth_status] < 0 || [string first {Complete} $impl_status] < 0} {
    fail "runs did not complete"
}

open_run impl_1
report_drc -checks {UCIO-1 NSTD-1}

set proj_dir [get_property DIRECTORY [current_project]]
puts "BIT=$proj_dir/$project_name.runs/impl_1/design_1_wrapper.bit"
puts "===== BUILD DONE ====="
exit 0
