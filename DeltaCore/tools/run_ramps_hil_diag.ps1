param(
    [string]$Port = "",
    [int]$HeaterPulseMs = 250
)
$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Runner = Join-Path $Here "run_ramps_hil_diag.py"

function Find-Python {
    foreach ($cmd in @("py", "python")) {
        try { & $cmd --version *> $null; if ($LASTEXITCODE -eq 0) { return $cmd } } catch {}
    }
    throw "Python 3 not found"
}
$Python = Find-Python
try { & $Python -c "import serial" *> $null } catch {}
if ($LASTEXITCODE -ne 0) { & $Python -m pip install --user pyserial }
$argsList = @($Runner, "--heater-pulse-ms", "$HeaterPulseMs")
if ($Port) { $argsList += @("--port", $Port) }
& $Python @argsList
exit $LASTEXITCODE
