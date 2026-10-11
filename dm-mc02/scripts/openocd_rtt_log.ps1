#!/usr/bin/env pwsh
# RM2026 PowerShell entry adapted to the shared MC-02 implementation.
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$pythonCommand = if ($env:PYTHON_BIN) { $env:PYTHON_BIN } elseif (Get-Command python3 -ErrorAction SilentlyContinue) { "python3" } elseif (Get-Command python -ErrorAction SilentlyContinue) { "python" } else { throw "Python 3.10+ is required (set PYTHON_BIN)." }
& $pythonCommand (Join-Path $PSScriptRoot "devtools.py") rtt @args
exit $LASTEXITCODE
