param(
    [string]$JetsonHost = "192.168.0.9",
    [string]$JetsonUser = "jetson",
    [string]$JetsonRoot = "/home/jetson/ultra_yubin",
    [string]$ProjectRoot = "C:\Users\hansung\examples\ultra_yubin",
    [string]$ProjectName = "ultra_yubin",
    [string]$VivadoSettings = "C:\Xilinx\Vivado\2023.1\settings64.bat",
    [bool]$Clean = $true
)

$ErrorActionPreference = "Stop"

function Die($Message) {
    Write-Error $Message
    exit 1
}

function Invoke-External($Exe, [string[]]$ArgsList, [int]$Retries = 1) {
    for ($i = 1; $i -le $Retries; $i++) {
        & $Exe @ArgsList
        if ($LASTEXITCODE -eq 0) { return }
        Write-Warning "$Exe failed with exit code $LASTEXITCODE ($i/$Retries)"
        if ($i -lt $Retries) { Start-Sleep -Seconds 2 }
    }
    Die "$Exe failed after $Retries attempt(s): $($ArgsList -join ' ')"
}

$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$LocalTools = Join-Path $ProjectRoot "codex_tools"
$BuildTcl = Join-Path $LocalTools "vivado_build_ultra_yubin.tcl"
$Rtl = Join-Path $ProjectRoot "rtl\pl_goal_compute_axi.v"
$Tb = Join-Path $ProjectRoot "tb\pl_goal_compute_axi_tb.v"
$Bit = Join-Path $ProjectRoot "$ProjectName.runs\impl_1\design_1_wrapper.bit"
$Log = Join-Path $LocalTools "vivado_ultra_yubin_$Stamp.log"
$JetsonTarget = "${JetsonUser}@${JetsonHost}"

New-Item -ItemType Directory -Force -Path $ProjectRoot | Out-Null
New-Item -ItemType Directory -Force -Path $LocalTools | Out-Null
New-Item -ItemType Directory -Force -Path (Split-Path $Rtl -Parent) | Out-Null
New-Item -ItemType Directory -Force -Path (Split-Path $Tb -Parent) | Out-Null

if (!(Test-Path $VivadoSettings)) { Die "Vivado settings not found: $VivadoSettings" }

Invoke-External "ssh" @($JetsonTarget, "mkdir -p ${JetsonRoot}/benchmark_logs ${JetsonRoot}/bitstream") 2

Write-Host "===== FETCH RTL / TCL FROM JETSON ====="
Invoke-External "scp" @("$($JetsonTarget):${JetsonRoot}/tools/windows/vivado_build_ultra_yubin.tcl", "$BuildTcl") 2
Invoke-External "scp" @("$($JetsonTarget):${JetsonRoot}/hardware/pl_goal_compute/rtl/pl_goal_compute_axi.v", "$Rtl") 2
Invoke-External "scp" @("$($JetsonTarget):${JetsonRoot}/hardware/pl_goal_compute/tb/pl_goal_compute_axi_tb.v", "$Tb") 2

if ($Clean) {
    Write-Host "===== CLEAN OLD VIVADO PROJECT OUTPUT ====="
    $Generated = @(
        (Join-Path $ProjectRoot "$ProjectName.xpr"),
        (Join-Path $ProjectRoot "$ProjectName.cache"),
        (Join-Path $ProjectRoot "$ProjectName.gen"),
        (Join-Path $ProjectRoot "$ProjectName.hw"),
        (Join-Path $ProjectRoot "$ProjectName.ip_user_files"),
        (Join-Path $ProjectRoot "$ProjectName.runs"),
        (Join-Path $ProjectRoot "$ProjectName.sim"),
        (Join-Path $ProjectRoot "$ProjectName.srcs"),
        (Join-Path $ProjectRoot ".Xil")
    )
    foreach ($Path in $Generated) {
        if (Test-Path $Path) {
            Remove-Item -Recurse -Force $Path
        }
    }
}

Write-Host "===== RUN VIVADO BUILD ====="
$VivadoCmd = "call `"$VivadoSettings`" && vivado -mode batch -source `"$BuildTcl`" -tclargs `"$ProjectRoot`" `"$ProjectName`" `"$Rtl`" `"$Tb`""
cmd.exe /c "$VivadoCmd" 2>&1 | Tee-Object -FilePath $Log
if ($LASTEXITCODE -ne 0) {
    Invoke-External "scp" @("$Log", "$($JetsonTarget):${JetsonRoot}/benchmark_logs/vivado_ultra_yubin_$Stamp.log") 2
    Die "Vivado build failed. Log: $Log"
}

if (!(Test-Path $Bit)) { Die "Bitstream not found after build: $Bit" }

$Hwh = Get-ChildItem $ProjectRoot -Recurse -Filter design_1.hwh |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

Write-Host "===== SEND ARTIFACTS TO JETSON ====="
Invoke-External "scp" @("$Bit", "$($JetsonTarget):${JetsonRoot}/bitstream/ultra_yubin.bit") 3
if ($null -ne $Hwh) {
    Invoke-External "scp" @("$($Hwh.FullName)", "$($JetsonTarget):${JetsonRoot}/bitstream/ultra_yubin.hwh") 3
}
Invoke-External "scp" @("$Log", "$($JetsonTarget):${JetsonRoot}/benchmark_logs/vivado_ultra_yubin_$Stamp.log") 3
Invoke-External "ssh" @($JetsonTarget, "ls -lh ${JetsonRoot}/bitstream/ultra_yubin.bit ${JetsonRoot}/bitstream/ultra_yubin.hwh 2>/dev/null || true; test -s ${JetsonRoot}/bitstream/ultra_yubin.bit") 2

Write-Host "===== DONE ====="
Write-Host "Next on Jetson:"
Write-Host "cd $JetsonRoot && ./tools/deploy_ultra96_ps_usb.sh"
