param(
    [Parameter(Mandatory=$true)]
    [string]$Port
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command pio -ErrorAction SilentlyContinue)) {
    throw "PlatformIO CLI ('pio') was not found. Install PlatformIO first."
}

Push-Location $PSScriptRoot
try {
    Write-Host "Building DeltaCore for Mega2560..."
    pio run -e megaatmega2560
    if ($LASTEXITCODE -ne 0) { throw "Build failed." }

    Write-Host "Uploading DeltaCore to $Port..."
    pio run -e megaatmega2560 -t upload --upload-port $Port
    if ($LASTEXITCODE -ne 0) { throw "Upload failed." }

    Write-Host ""
    Write-Host "FLASH COMPLETE"
    Write-Host "Open serial at 250000 baud and send M119 FIRST."
    Write-Host "Do NOT send G28 until A/B/C MAX switches are verified manually."
}
finally {
    Pop-Location
}
