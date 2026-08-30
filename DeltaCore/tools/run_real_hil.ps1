param(
    [string]$Port = "",
    [int]$Rounds = 10,
    [switch]$Setup,
    [switch]$NoRaw
)

$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Runner = Join-Path $Here "real_hil.py"

function Find-Python {
    foreach ($cmd in @("py", "python")) {
        try {
            $null = & $cmd --version 2>$null
            if ($LASTEXITCODE -eq 0) { return $cmd }
        } catch {}
    }
    throw "Python 3 not found. Install Python 3 and rerun."
}

$Python = Find-Python
Write-Host "DeltaCore REAL-HIL" -ForegroundColor Cyan
Write-Host "Python: $Python"

try {
    & $Python -c "import serial" 2>$null
    if ($LASTEXITCODE -ne 0) { throw "missing" }
} catch {
    Write-Host "Installing pyserial..."
    & $Python -m pip install --user pyserial
    if ($LASTEXITCODE -ne 0) { throw "Failed to install pyserial" }
}

$argsList = @($Runner, "--rounds", "$Rounds")
if ($Port) { $argsList += @("--port", $Port) }
if ($Setup) { $argsList += "--setup" }
if ($NoRaw) { $argsList += "--no-raw" }

Write-Host ""
Write-Host "Running: $Python $($argsList -join ' ')" -ForegroundColor DarkGray
& $Python @argsList
$code = $LASTEXITCODE

Write-Host ""
if ($code -eq 0) {
    Write-Host "REAL-HIL PASS" -ForegroundColor Green
} else {
    Write-Host "REAL-HIL FAIL - inspect real_hil_logs" -ForegroundColor Red
}
exit $code
