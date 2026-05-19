# Vivado report helper: run PL goal-compute RTL simulation and open a clean
# report waveform with only bbox input and computed pan/tilt goal signals.
# Usage in Vivado Tcl Console:
#   source C:/Users/hansung/examples/ultra_yubin_v1/tools/vivado_pl_goal_waveform.tcl

if {[catch {current_project} proj_name]} {
    error "Open ultra_yubin_v1.xpr first, then source this script."
}

set script_path [file normalize [info script]]
set repo_dir [file dirname [file dirname $script_path]]
set proj_dir [get_property DIRECTORY [current_project]]

proc ensure_file_from_github {path url} {
    if {[file exists $path]} {
        return
    }
    puts "Missing: $path"
    puts "Downloading: $url"
    file mkdir [file dirname $path]
    if {[catch {
        exec powershell -NoProfile -ExecutionPolicy Bypass -Command "Invoke-WebRequest -Uri '$url' -OutFile '$path'"
    } err]} {
        puts "WARN: automatic download failed: $err"
    }
}

ensure_file_from_github \
    [file join $proj_dir tools vivado_pl_goal_waveform.tcl] \
    "https://raw.githubusercontent.com/binibini1112/ultra_yubin_v1/main/tools/vivado_pl_goal_waveform.tcl"
ensure_file_from_github \
    [file join $proj_dir hardware pl_goal_compute rtl pl_goal_compute_axi.v] \
    "https://raw.githubusercontent.com/binibini1112/ultra_yubin_v1/main/hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v"
ensure_file_from_github \
    [file join $proj_dir hardware pl_goal_compute tb pl_goal_compute_axi_tb.v] \
    "https://raw.githubusercontent.com/binibini1112/ultra_yubin_v1/main/hardware/pl_goal_compute/tb/pl_goal_compute_axi_tb.v"

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

set wave_name "goal_only_[clock seconds]"
catch {create_wave_config $wave_name}

proc add_report_wave {sig} {
    if {[catch {add_wave -radix decimal $sig} err]} {
        puts "WARN: add_wave failed for $sig: $err"
    }
}

add_report_wave /pl_goal_compute_axi_tb/dut/track_cx
add_report_wave /pl_goal_compute_axi_tb/dut/track_cy
add_report_wave /pl_goal_compute_axi_tb/dut/track_fw
add_report_wave /pl_goal_compute_axi_tb/dut/track_fh
add_report_wave /pl_goal_compute_axi_tb/dut/track_error_x
add_report_wave /pl_goal_compute_axi_tb/dut/track_error_y
add_report_wave /pl_goal_compute_axi_tb/dut/track_pan_step
add_report_wave /pl_goal_compute_axi_tb/dut/track_tilt_step
add_report_wave /pl_goal_compute_axi_tb/dut/pan_goal
add_report_wave /pl_goal_compute_axi_tb/dut/tilt_goal
add_report_wave /pl_goal_compute_axi_tb/dut/last_pan
add_report_wave /pl_goal_compute_axi_tb/dut/last_tilt

run all
catch {wave zoom range 900ns 2600ns}

puts "Done. Check Tcl Console for TB_PASS and capture the $wave_name waveform window."
