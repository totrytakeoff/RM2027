#!/usr/bin/env pwsh
# PowerShell version of openocd_rtt_log.sh
# Requires: OpenOCD, netcat (nc)

$ErrorActionPreference = "Stop"

$ScriptRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$RootDir = (Get-Item $ScriptRoot).FullName

# Configuration with environment variable defaults
if ([string]::IsNullOrEmpty($env:OPENOCD_CFG)) { $env:OPENOCD_CFG = "$RootDir\platform\boards\rm_f407\openocd\openocd_dap.cfg" }
$OpenOCD_Cfg = $env:OPENOCD_CFG
if ([string]::IsNullOrEmpty($env:OPENOCD_HOST)) { $env:OPENOCD_HOST = "127.0.0.1" }
$OpenOCD_Host = $env:OPENOCD_HOST
if ([string]::IsNullOrEmpty($env:OPENOCD_TELNET_PORT)) { $env:OPENOCD_TELNET_PORT = "4444" }
$OpenOCD_TelnetPort = [int]$env:OPENOCD_TELNET_PORT
if ([string]::IsNullOrEmpty($env:OPENOCD_TCL_PORT)) { $env:OPENOCD_TCL_PORT = "6666" }
$OpenOCD_TclPort = [int]$env:OPENOCD_TCL_PORT
if ([string]::IsNullOrEmpty($env:OPENOCD_RESET_RUN)) { $env:OPENOCD_RESET_RUN = "1" }
$OpenOCD_ResetRun = $env:OPENOCD_RESET_RUN

# RTT Configuration
if ([string]::IsNullOrEmpty($env:RTT_RAM_ADDR)) { $env:RTT_RAM_ADDR = "0x20000000" }
$RTT_RamAddr = $env:RTT_RAM_ADDR
if ([string]::IsNullOrEmpty($env:RTT_RAM_SIZE)) { $env:RTT_RAM_SIZE = "0x20000" }
$RTT_RamSize = $env:RTT_RAM_SIZE
if ([string]::IsNullOrEmpty($env:RTT_NAME)) { $env:RTT_NAME = "SEGGER RTT" }
$RTT_Name = $env:RTT_NAME

if ([string]::IsNullOrEmpty($env:RTT_SERVER_PORT)) { $env:RTT_SERVER_PORT = "19021" }
$RTT_ServerPort = [int]$env:RTT_SERVER_PORT
if ([string]::IsNullOrEmpty($env:RTT_CHANNEL)) { $env:RTT_CHANNEL = "0" }
$RTT_Channel = [int]$env:RTT_CHANNEL

$Global:OpenOCD_Process = $null
$Global:OpenOCD_Path = $null
# Use Windows temp directory
$OpenOCD_LogFile = Join-Path $env:TEMP "openocd-rtt.log"

function Find-OpenOCD {
    param([switch]$Silent)

    # Try to find PlatformIO's openocd first
    $pioPaths = @(
        "$env:LOCALAPPDATA\Programs\Microsoft VS Code\data\user-data\Extensions\platformio.platformio-ide*\latest\tools\bin\openocd.exe",
        "$env:USERPROFILE\.platformio\packages\tool-openocd\bin\openocd.exe",
        "$env:ProgramFiles\PlatformIO\core\penv\Scripts\openocd.exe"
    )

    foreach ($pattern in $pioPaths) {
        $resolved = Resolve-Path $pattern -ErrorAction SilentlyContinue
        if ($resolved -and (Test-Path $resolved.Path)) {
            if (-not $Silent) { Write-Host "[rtt] Found PlatformIO OpenOCD: $($resolved.Path)" }
            return $resolved.Path
        }
    }

    # Try to find via Get-Command if it exists
    try {
        $cmd = Get-Command "openocd" -ErrorAction Stop
        if (-not $Silent) { Write-Host "[rtt] Found system OpenOCD: $($cmd.Source)" }
        return $cmd.Source
    } catch {
        return $null
    }
}

function Test-Command {
    param([string]$Name)
    try {
        $null = Get-Command $Name -ErrorAction Stop
        return $true
    } catch {
        return $false
    }
}

function Test-PortOpen {
    param([int]$Port)
    try {
        $socket = New-Object System.Net.Sockets.TcpClient
        $socket.Connect($OpenOCD_Host, $Port)
        $socket.Close()
        return $true
    } catch {
        return $false
    }
}

function Wait-Port {
    param(
        [int]$Port,
        [string]$Label,
        [int]$MaxTries = 100
    )
    for ($i = 0; $i -lt $MaxTries; $i++) {
        if (Test-PortOpen -Port $Port) {
            return $true
        }
        Start-Sleep -Milliseconds 50
    }
    Write-Error "[rtt] timeout waiting for ${Label} on ${OpenOCD_Host}:${Port}"
}

function Stop-OpenOCD {
    if ($Global:OpenOCD_Process -and -not $Global:OpenOCD_Process.HasExited) {
        try {
            $Global:OpenOCD_Process.Kill()
            $Global:OpenOCD_Process.WaitForExit(2000)
        } catch {
            # Ignore errors during cleanup
        }
    }
}

function Start-OpenOCD-If-Needed {
    if (Test-PortOpen -Port $OpenOCD_TelnetPort) {
        Write-Host "[rtt] OpenOCD already running on ${OpenOCD_Host}:${OpenOCD_TelnetPort}"
        return
    }

    # Find OpenOCD (prefer PlatformIO's version)
    $Global:OpenOCD_Path = Find-OpenOCD
    if (-not $Global:OpenOCD_Path) {
        Write-Error "missing required command: openocd. Please install OpenOCD or PlatformIO."
    }

    if (-not (Test-Path $OpenOCD_Cfg)) {
        Write-Error "[rtt] OpenOCD cfg not found: ${OpenOCD_Cfg}"
    }

    Write-Host "[rtt] starting OpenOCD: $($Global:OpenOCD_Path) -f ${OpenOCD_Cfg}"

    # Use different files for stdout and stderr
    $stdOutFile = $OpenOCD_LogFile
    $stdErrFile = $OpenOCD_LogFile + ".err"
    Write-Host "[rtt] OpenOCD log: ${stdOutFile}"

    $process = Start-Process -FilePath $Global:OpenOCD_Path -ArgumentList "-f", $OpenOCD_Cfg -PassThru -RedirectStandardOutput $stdOutFile -RedirectStandardError $stdErrFile -NoNewWindow
    $Global:OpenOCD_Process = $process

    if (-not (Wait-Port -Port $OpenOCD_TelnetPort -Label "OpenOCD telnet")) {
        Write-Error "[rtt] OpenOCD did not open telnet port ${OpenOCD_TelnetPort}"
    }
}

function Send-OpenOCD-Telnet-Cmds {
    param([string]$Cmds)

    # Try Python first, fall back to netcat
    if (Test-Command "python3") {
        $pythonScript = @"
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
cmds = sys.stdin.read()

s = socket.create_connection((host, port), timeout=2.0)
s.settimeout(1.0)
time.sleep(0.05)
try:
    s.recv(4096)
except Exception:
    pass

s.sendall(cmds.encode("utf-8"))
time.sleep(0.05)

out = b""
try:
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        out += chunk
        if b"\n>" in out or b"unknown command" in out:
            break
except Exception:
    pass

s.close()
sys.stdout.write(out.decode("utf-8", errors="ignore"))
"@

        $result = python3 -c $pythonScript $OpenOCD_Host $OpenOCD_TelnetPort -WindowStyle Hidden -ArgumentList $Cmds
        return
    }

    if (-not (Test-Command "nc")) {
        Write-Error "missing required command: nc (netcat)"
    }

    $result = echo $Cmds | nc $OpenOCD_Host $OpenOCD_TelnetPort
    return $result
}

function Send-OpenOCD-Tcl-Cmd {
    param([string]$Cmd)

    if (-not $Cmd.EndsWith("`n")) {
        $Cmd += "`n"
    }

    $term = [char]0x1a
    $payload = $Cmd + $term

    # Use .NET socket directly for reliable TCL communication
    try {
        $client = New-Object System.Net.Sockets.TcpClient
        $client.ReceiveTimeout = 2000
        $client.SendTimeout = 2000
        $client.Connect($OpenOCD_Host, $OpenOCD_TclPort)

        $stream = $client.GetStream()
        $writer = New-Object System.IO.StreamWriter($stream)
        $reader = New-Object System.IO.StreamReader($stream)

        $writer.AutoFlush = $true

        # Send command with Ctrl-Z terminator
        $writer.Write($payload)
        $writer.Flush()

        # Read response until terminator
        $response = $reader.ReadToEnd()
        $client.Close()

        return $response
    } catch {
        Write-Warning "[rtt] TCL communication error: $_"
        return ""
    }
}

function Reset-Run-If-Enabled {
    if ($OpenOCD_ResetRun -ne "1") {
        return
    }

    Write-Host "[rtt] reset run (OPENOCD_RESET_RUN=1)"
    $null = Send-OpenOCD-Tcl-Cmd "reset run"
    Start-Sleep -Milliseconds 200
}

function Setup-RTT {
    Write-Host "[rtt] configuring RTT (addr=${RTT_RamAddr}, size=${RTT_RamSize}, name=${RTT_Name})"

    $cmds = "rtt setup ${RTT_RamAddr} ${RTT_RamSize} `"${RTT_Name}`"
rtt start
rtt server start ${RTT_ServerPort} ${RTT_Channel}
"
    Write-Host "[rtt] sending RTT commands via TCL port ${OpenOCD_TclPort}..."
    $out = Send-OpenOCD-Tcl-Cmd $cmds

    Write-Host "[rtt] TCL response: '$out'"

    if ($out) {
        Write-Host $out
    }

    if ($out -match "unknown command|invalid command name .rtt.|no such command") {
        Write-Error "[rtt] ERROR: your OpenOCD does not support RTT commands (missing rtt).`nTry a newer OpenOCD build with RTT enabled, or switch logs to UART/SWO."
    }

    # Wait a bit for RTT server to start
    Start-Sleep -Milliseconds 500

    # Check if RTT server port is open
    if (Test-PortOpen -Port $RTT_ServerPort) {
        Write-Host "[rtt] RTT server is running on port ${RTT_ServerPort}"
    } else {
        Write-Warning "[rtt] RTT server did not start on port ${RTT_ServerPort}"
        Write-Warning "[rtt] Check OpenOCD log for details"
    }
}

function Find-Netcat {
    # Try common netcat locations on Windows
    $ncPaths = @(
        "$env:ProgramFiles\Git\usr\bin\nc.exe",
        "$env:ProgramFiles\Git\usr\bin\ncat.exe",
        "$env:ProgramFiles\Netcat\nc.exe",
        "$env:SystemRoot\System32\nc.exe"
    )

    foreach ($path in $ncPaths) {
        if (Test-Path $path) {
            return $path
        }
    }

    # Try via Get-Command
    foreach ($name in @("nc", "ncat", "netcat")) {
        try {
            $cmd = Get-Command $name -ErrorAction Stop
            return $cmd.Source
        } catch {
            continue
        }
    }

    return $null
}

function Connect-RTT {
    # Try to find netcat
    $ncPath = Find-Netcat

    if ($ncPath) {
        if (-not (Wait-Port -Port $RTT_ServerPort -Label "RTT server")) {
            Write-Error "[rtt] RTT server port ${RTT_ServerPort} is not listening."
        }

        Write-Host "[rtt] connecting: nc ${OpenOCD_Host} ${RTT_ServerPort}  (Ctrl-C to exit)"
        & $ncPath $OpenOCD_Host $RTT_ServerPort
        return
    }

    # Fallback: Use PowerShell TCP client to read RTT data
    Write-Host "[rtt] netcat not found, using PowerShell TCP client (Ctrl-C to exit)"
    Write-Host "[rtt] connecting to ${OpenOCD_Host}:${RTT_ServerPort}"

    try {
        $client = New-Object System.Net.Sockets.TcpClient
        $client.Connect($OpenOCD_Host, $RTT_ServerPort)
        $stream = $client.GetStream()
        $buffer = New-Object byte[] 4096

        while ($client.Connected) {
            if ($stream.DataAvailable) {
                $bytesRead = $stream.Read($buffer, 0, $buffer.Length)
                if ($bytesRead -gt 0) {
                    [System.Console]::Out.Write([System.Text.Encoding]::ASCII.GetString($buffer, 0, $bytesRead))
                }
            }
            Start-Sleep -Milliseconds 10
        }

        $client.Close()
    } catch {
        Write-Error "[rtt] Failed to connect to RTT server: $_"
    }
}

function Main {
    # Register cleanup handler
    Register-EngineEvent -SourceIdentifier "PowerShell.Exiting" -Action { Stop-OpenOCD } | Out-Null

    try {
        Start-OpenOCD-If-Needed
        Reset-Run-If-Enabled
        Setup-RTT
        Connect-RTT
    } finally {
        Stop-OpenOCD
    }
}

Main @args
                }
