param(
    [string]$Hex          = "${env:USERPROFILE}\eclipse-workspace\M2_HW\Debug\M2_HW.hex",
    [string]$AvrDude      = "C:\TOOLS\WinAVR-20100110\bin\avrdude.exe",
    [string]$Port         = "COM5",
    [int]   $UploadBaud   = 115200,
    [string]$PuTTY        = "C:\Program Files\PuTTY\putty.exe",
    [string]$PuTTYSession = "Segway",
    [string]$PuTTYTitle   = "Segway"
)

$PSStyle.OutputRendering = 'PlainText'
$ErrorActionPreference = "Stop"

function Stop-PuTTYByTitle {
    param(
        [string]$WindowTitle
    )

    $procs = Get-Process -Name putty -ErrorAction SilentlyContinue |
        Where-Object { $_.MainWindowTitle -eq $WindowTitle }

    if (-not $procs) {
        Write-Host "No PuTTY window with title '$WindowTitle' found."
        return
    }

    foreach ($p in $procs) {
        try {
            Stop-Process -Id $p.Id -Force -ErrorAction Stop
            Write-Host "Stopped PuTTY window '$WindowTitle' (PID $($p.Id))"
        }
        catch {
            Write-Warning "Failed to stop PuTTY PID $($p.Id): $($_.Exception.Message)"
        }
    }
}

if (-not (Test-Path -LiteralPath $Hex)) {
    throw "HEX file not found: $Hex"
}

if (-not (Test-Path -LiteralPath $AvrDude)) {
    throw "avrdude not found: $AvrDude"
}

if (-not (Test-Path -LiteralPath $PuTTY)) {
    throw "PuTTY not found: $PuTTY"
}

$hexDir  = Split-Path -Parent $Hex
$hexFile = Split-Path -Leaf   $Hex

Write-Host "Stopping existing serial monitor ..."
Stop-PuTTYByTitle -WindowTitle $PuTTYTitle

Start-Sleep -Milliseconds 300

Push-Location $hexDir
try {
    $avrdudeArgs = @(
        "-pm328p"
        "-carduino"
        "-P//./$Port"
        "-b$UploadBaud"
        "-Uflash:w:${hexFile}:a"
    )

    Write-Host ""
    Write-Host "Flashing $Hex to $Port ..."
    Write-Host ""
    Write-Host "$AvrDude $($avrdudeArgs -join ' ')"
    Write-Host ""

	& $AvrDude @avrdudeArgs 2>&1 | ForEach-Object {
	    [Console]::Out.WriteLine($_.ToString())
	}
	$rc = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($rc -ne 0) {
    Write-Host ""
    Write-Host "FLASH FAILED (rc=$rc)"
    exit $rc
}

Write-Host ""
Write-Host "FLASH OK"
Write-Host ""
Write-Host "Reopening serial monitor using saved session '$PuTTYSession' ..."

Start-Sleep -Milliseconds 300

Start-Process -FilePath $PuTTY -ArgumentList @(
    "-load", $PuTTYSession
) | Out-Null

exit 0