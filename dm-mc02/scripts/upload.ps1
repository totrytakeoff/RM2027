#!/usr/bin/env pwsh
# =============================================================================
# @file scripts/upload.ps1
# @brief STM32 OpenOCD PowerShell 烧录脚本
# @project RM2027 / DM MC-02 (adapted from RM2026)
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 解析 build 目录产物路径，调用 OpenOCD 进行 flash/verify，可定制地址、格式和配置文件。
# =============================================================================

# 设置严格模式
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# 设置控制台编码为UTF-8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$OutputEncoding = [System.Text.Encoding]::UTF8

# 定义常量
$RootDir = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$BuildDir = Join-Path $RootDir "build"
$Preset = "h7-debug"
$OutputDir = Join-Path $BuildDir $Preset
$Backend = "dapv2"
$Speed = ""
$VerboseFlash = $false
$DefaultFile = "chassis_demo"
$DefaultFormat = "bin"
$DefaultAddr = "0x08000000"
$DefaultCfg = Join-Path $RootDir "platform/boards/dm_mc02/openocd/cmsis-dap.cfg"

# 帮助信息
function Show-Usage {
    Write-Host 'Usage: ./scripts/upload.ps1 [name] [--file path] [--format bin|hex|elf] [--addr 0x...] [--cfg path] [--openocd path] [--verify-only] [--dry-run]'
    Write-Host 'Optional: --preset h7-debug|h7-release|h7-dbus-debug|h7-chassis-observe|h7-mecanum-observe --backend dapv2|dapv1|auto --speed kHz --verbose'
    Write-Host 'Default: build/h7-debug/chassis_demo.bin, DAP v2. Requires PowerShell and OpenOCD only.'
}

# 初始化变量
$File = $DefaultFile
$Format = $DefaultFormat
$Addr = ""
$Cfg = $DefaultCfg
$OpenOcdBin = ""
$VerifyOnly = $false
$DryRun = $false
$PositionalFile = ""

# 解析命令行参数
$i = 0
while ($i -lt $args.Length) {
    if ($args[$i] -in @("--file","--format","--addr","--address","--cfg","--config","--openocd","--preset","--backend","--speed")) {
        if ($i + 1 -ge $args.Length -or [string]::IsNullOrEmpty($args[$i+1])) { throw "Missing value: $($args[$i])" }
    }
    switch ($args[$i]) {
        "--preset" { $Preset = $args[$i+1]; $i += 2 }
        "--backend" { $Backend = $args[$i+1]; $i += 2 }
        "--speed" { $Speed = $args[$i+1]; $i += 2 }
        "--verbose" { $VerboseFlash = $true; $i += 1 }
        "--file" {
            $File = $args[$i+1]
            $i += 2
        }
        "--format" {
            $Format = $args[$i+1]
            $i += 2
        }
        "--addr" {
            $Addr = $args[$i+1]
            $i += 2
        }
        "--address" {
            $Addr = $args[$i+1]
            $i += 2
        }
        "--cfg" {
            $Cfg = $args[$i+1]
            $i += 2
        }
        "--config" {
            $Cfg = $args[$i+1]
            $i += 2
        }
        "--openocd" {
            $OpenOcdBin = $args[$i+1]
            $i += 2
        }
        "--verify-only" {
            $VerifyOnly = $true
            $i += 1
        }
        "--dry-run" {
            $DryRun = $true
            $i += 1
        }
        "-h" {
            Show-Usage; exit 0
        }
        "--help" {
            Show-Usage; exit 0
        }
        default {
            if ($args[$i].StartsWith("--")) { throw "Unknown argument: $($args[$i])" }
            if ([string]::IsNullOrEmpty($PositionalFile)) {
                $PositionalFile = $args[$i]
                $i += 1
            } else {
                Write-Host "未知参数: $($args[$i])" -ForegroundColor Red
                Show-Usage; exit 1
            }
        }
    }
}

if ($Preset -notin @("h7-debug","h7-release","h7-dbus-debug","h7-chassis-observe","h7-mecanum-observe")) { throw "Unsupported firmware preset: $Preset" }
$OutputDir = Join-Path $BuildDir $Preset
if ($Preset -eq "h7-mecanum-observe" -and $File -eq $DefaultFile -and -not $PositionalFile) { $File = "mecanum_demo" }
if ($Preset -eq "h7-dbus-debug" -and $File -eq $DefaultFile -and -not $PositionalFile) { $File = "board_bringup" }
if ($Backend -notin @("dapv2","dapv1","auto")) { throw "Invalid backend: $Backend" }
if ($Speed -and $Speed -notmatch '^[1-9][0-9]*$') { throw "speed must be positive kHz" }
if ($Addr -and $Addr -notmatch '^(0[xX][0-9a-fA-F]+|[0-9]+)$') { throw "Invalid image address" }

# 处理位置参数
if (-not [string]::IsNullOrEmpty($PositionalFile)) {
    $File = $PositionalFile
}

# 验证格式
if ($Format -notin ("bin", "hex", "elf")) {
    Write-Host "格式必须为 bin|hex|elf" -ForegroundColor Red
    exit 1
}

# 解析配置文件路径
if ($Cfg -like "~/*") { $Cfg = Join-Path $HOME $Cfg.Substring(2) }
if (-not ([IO.Path]::IsPathRooted($Cfg)) -and -not ($Cfg -like "~/*")) {
    $Cfg = Join-Path $RootDir $Cfg
}

# 解析目标文件路径
function Resolve-TargetFile {
    param([string]$Path)
    $expanded = $Path
    
    # 处理 ~ 路径
    if ($expanded -like "~/*") {
        $expanded = Join-Path $HOME $expanded.Substring(2)
    } elseif ($expanded -eq "~") {
        $expanded = $HOME
    }
    
    # 处理绝对路径
    if ([IO.Path]::IsPathRooted($expanded)) {
        if ([IO.Path]::HasExtension($expanded)) {
            return $expanded
        } else {
            return "$expanded.$Format"
        }
    }

    # 现存相对文件优先按调用者当前目录解析；否则仍按产物简称到 build/output 查找。
    $relativeCandidate = if ([IO.Path]::HasExtension($expanded)) {
        $expanded
    } else {
        "$expanded.$Format"
    }
    if (Test-Path -LiteralPath $relativeCandidate -PathType Leaf) {
        return (Resolve-Path -LiteralPath $relativeCandidate).Path
    }
    
    $rootCandidate = Join-Path $RootDir $relativeCandidate
    if (Test-Path -LiteralPath $rootCandidate -PathType Leaf) { return $rootCandidate }
    if ($expanded.Contains('/') -or $expanded.Contains('\')) { return $rootCandidate }
    # 处理相对路径
    if ([IO.Path]::HasExtension($expanded)) {
        return Join-Path $OutputDir $expanded
    } else {
        return Join-Path $OutputDir "$expanded.$Format"
    }
}

$TargetFile = Resolve-TargetFile $File
if (-not (Test-Path $TargetFile)) {
    Write-Host "未找到文件: $TargetFile" -ForegroundColor Red
    exit 1
}

$ImageType = [IO.Path]::GetExtension($TargetFile).TrimStart('.').ToLowerInvariant()
if ($ImageType -notin ("bin", "hex", "elf")) {
    Write-Host "无法识别镜像类型: $TargetFile" -ForegroundColor Red
    exit 1
}
if ($ImageType -eq "bin" -and [string]::IsNullOrEmpty($Addr)) {
    $Addr = $DefaultAddr
}

# 查找 OpenOCD 可执行文件
if ($OpenOcdBin) {
    $resolvedCommand = Get-Command $OpenOcdBin -ErrorAction SilentlyContinue
    if ($resolvedCommand) { $OpenOcdBin = $resolvedCommand.Source }
}
if ([string]::IsNullOrEmpty($OpenOcdBin)) {
    $openocdCommand = Get-Command openocd -ErrorAction SilentlyContinue
    if ($openocdCommand -ne $null) {
        $OpenOcdBin = $openocdCommand.Source
    } else {
        $OpenOcdBin = Join-Path $env:USERPROFILE ".platformio\packages\tool-openocd\bin\openocd.exe"
    }
}

if (-not (Test-Path $OpenOcdBin)) {
    Write-Host "OpenOCD 不可用: $OpenOcdBin" -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $Cfg)) {
    Write-Host "OpenOCD 配置不存在: $Cfg" -ForegroundColor Red
    exit 1
}

# Build native OpenOCD arguments; no Python runtime.
function Quote-TclPath {
    param([string]$Value)
    $Value = $Value.Replace('\','/').Replace('"','\"').Replace('$','\$').Replace('[','\[').Replace(']','\]').Replace('{','\{').Replace('}','\}')
    $Value = $Value.Replace("`n", '\n').Replace("`r", '\r')
    return '"' + $Value + '"'
}
$cmdArgs = @("-f", $Cfg)
if ($Backend -eq "dapv2") { $cmdArgs += @("-c", "cmsis_dap_backend usb_bulk") }
if ($Backend -eq "dapv1") { $cmdArgs += @("-c", "cmsis_dap_backend hid") }
if ($Speed) { $cmdArgs += @("-c", "adapter speed $Speed") }
if ($VerboseFlash) { $cmdArgs += @("-d", "3") }
$ImageSpec = Quote-TclPath $TargetFile
if ($Addr) { $ImageSpec += " $Addr $ImageType" }
$Operation = if ($VerifyOnly) { "verify_image $ImageSpec" } else { "flash write_image erase $ImageSpec; verify_image $ImageSpec" }
$cmdArgs += @("-c", "init; reset halt; if {[catch {$Operation} flash_error]} {puts stderr `$flash_error; shutdown error}; reset run; shutdown")

# 调试输出参数
Write-Host "参数列表: $(($cmdArgs | ForEach-Object { '"' + $_ + '"' }) -join ' ')" -ForegroundColor Cyan

# 输出信息
Write-Host "OpenOCD: $OpenOcdBin"
Write-Host "Config : $Cfg"
Write-Host "File   : $TargetFile"
Write-Host "Addr   : $(if ([string]::IsNullOrEmpty($Addr)) { '<from-image>' } else { $Addr })"
Write-Host "Mode   : $(if ($VerifyOnly) { 'verify-only' } else { 'flash+verify' })"

# 执行命令
if ($DryRun) {
    Write-Host "Command: $OpenOcdBin $($cmdArgs -join ' ' )"
    exit 0
}

# 执行 OpenOCD 命令
Write-Host "正在执行 OpenOCD 命令..." -ForegroundColor Yellow

# 直接使用调用操作符执行命令，确保参数数组正确传递
& $OpenOcdBin $cmdArgs

# 检查执行结果
if ($LASTEXITCODE -ne 0) {
    Write-Host "OpenOCD failed, code=$LASTEXITCODE" -ForegroundColor Red
    exit $LASTEXITCODE
} else {
    Write-Host "OpenOCD executed successfully!" -ForegroundColor Green
}
