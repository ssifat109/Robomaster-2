$ErrorActionPreference = 'Stop'
$robotProfile = 'RMEP-21b41f'
Write-Host "Connecting Wi-Fi to $robotProfile (using the saved profile)..."
netsh wlan connect "name=$robotProfile" 'interface=Wi-Fi' | Out-Host
if ($LASTEXITCODE -ne 0) { Write-Host 'Turn on the robot hotspot and connect to it in Windows Wi-Fi.'; exit 1 }
$ready = $false
for ($attempt = 0; $attempt -lt 20; $attempt++) {
    $addresses = Get-NetIPAddress -InterfaceAlias 'Wi-Fi' -AddressFamily IPv4 -ErrorAction SilentlyContinue
    if ($addresses | Where-Object { $_.IPAddress -like '192.168.2.*' -and $_.AddressState -eq 'Preferred' }) {
        $ready = $true
        break
    }
    Start-Sleep -Seconds 1
}
if (-not $ready) { Write-Host 'Robot Wi-Fi did not provide an address. Check its power and AP mode.'; exit 1 }
Write-Host 'Robot Wi-Fi address is ready.'
exit 0
