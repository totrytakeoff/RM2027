#!/usr/bin/env pwsh
# =============================================================================
# @file scripts/upload.ps1
# @brief STM32 OpenOCD PowerShell 烧录脚本
# @project RM2026
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
$OutputDir = Join-Path $BuildDir "output"
$DefaultFile = "app"
$DefaultFormat = "bin"
$DefaultAddr = "0x08000000"
$DefaultCfg = Join-Path $RootDir "platform/boards/rm_f407/openocd/openocd_dap.cfg"

# 帮助信息
function Show-Usage {
    Write-Host "用法: $((Split-Path -Leaf $MyInvocation.ScriptName)) [<name>] [--format bin|hex|elf] [--addr <0x...>] [--cfg <openocd.cfg>] [--openocd <path>] [--verify-only] [--dry-run]"
    Write-Host "说明: BIN 默认写入 0x08000000；HEX/ELF 默认使用文件内地址，--addr 对其表示地址偏移。"
    Write-Host "示例:"
    Write-Host "  $((Split-Path -Leaf $MyInvocation.ScriptName))                      # 烧录 build/output/app.bin 到 0x08000000"
    Write-Host "  $((Split-Path -Leaf $MyInvocation.ScriptName)) test_motor --format elf"
    Write-Host "  $((Split-Path -Leaf $MyInvocation.ScriptName)) --format hex --verify-only  # 默认使用 app.hex"
    Write-Host "  $((Split-Path -Leaf $MyInvocation.ScriptName)) test_remote_control_demo    # 上传 build/output/test_remote_control_demo.bin"
    Write-Host "  $((Split-Path -Leaf $MyInvocation.ScriptName)) build/output/app.bin        # 直接使用当前目录下的相对路径"
    Write-Host "  $((Split-Path -Leaf $MyInvocation.ScriptName)) /abs/path/to/custom.elf     # 直接指定绝对路径并保持扩展名"
    exit 0
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
    switch ($args[$i]) {
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
            Show-Usage
        }
        "--help" {
            Show-Usage
        }
        default {
            if ([string]::IsNullOrEmpty($PositionalFile)) {
                $PositionalFile = $args[$i]
                $i += 1
            } else {
                Write-Host "未知参数: $($args[$i])" -ForegroundColor Red
                Show-Usage
            }
        }
    }
}

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

# 构建命令参数数组
$cmdArgs = @(
    "-f", $Cfg,
    "-c", "init; reset halt"
)

# 确保路径使用正斜杠，避免PowerShell路径解析问题
$TargetFileForOpenOCD = $TargetFile.Replace('\', '/')
$ImageSpec = $TargetFileForOpenOCD
if (-not [string]::IsNullOrEmpty($Addr)) {
    $ImageSpec += " $Addr $ImageType"
}

if ($VerifyOnly) {
    $cmdArgs += @("-c", "verify_image $ImageSpec; reset; shutdown")
} else {
    $cmdArgs += @("-c", "flash write_image erase $ImageSpec; verify_image $ImageSpec; reset; shutdown")
}

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
