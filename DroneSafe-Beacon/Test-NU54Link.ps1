param(
    [Parameter(Mandatory)][ValidatePattern('^COM[1-9][0-9]*$')][string]$DronePort,
    [Parameter(Mandatory)][ValidatePattern('^COM[1-9][0-9]*$')][string]$SearcherPort,
    [Parameter(Mandatory)][ValidatePattern('^[A-Fa-f0-9]{8,128}$')][string]$DroneUsbId,
    [Parameter(Mandatory)][ValidatePattern('^[A-Fa-f0-9]{8,128}$')][string]$SearcherUsbId,
    [ValidateRange(1,60)][int]$Seconds = 20,
    [ValidateSet('','s','b','r','p','m','drop','SP STATUS','SP OPEN','SP CLOSE')][string]$DroneCommand = 's',
    [ValidateSet('','s','drop','retry','pause','resume','SP STATUS','SP OPEN','SP CLOSE')][string]$SearcherCommand = 's'
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$ports = @()
try {
    if ($DronePort -ieq $SearcherPort -or $DroneUsbId -ieq $SearcherUsbId) {
        throw 'The drone and searcher must be different USB devices.'
    }
    $specs = @(
        @{ Port=$DronePort; Id=$DroneUsbId; Role='drone'; Command=$DroneCommand },
        @{ Port=$SearcherPort; Id=$SearcherUsbId; Role='searcher'; Command=$SearcherCommand }
    )
    foreach ($spec in $specs) {
        $dev = @(Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq $spec.Port)
        if ($dev.Count -ne 1) { throw "Missing/ambiguous $($spec.Port)" }
        $instance = [string]$dev[0].PNPDeviceID
        $matched = $false
        for ($depth=0; $depth -lt 8 -and $instance; $depth++) {
            if (($instance -split '\\')[-1] -ieq $spec.Id) { $matched=$true; break }
            $instance = [string](Get-PnpDeviceProperty -InstanceId $instance -KeyName DEVPKEY_Device_Parent).Data
        }
        if (!$matched) { throw "Identity mismatch on $($spec.Port)" }
        $serial = [System.IO.Ports.SerialPort]::new($spec.Port,115200)
        $serial.NewLine="`n"
        $serial.ReadTimeout=50
        $serial.WriteTimeout=1000
        $serial.DtrEnable=$true
        try { $serial.Open() } catch { $serial.Dispose(); throw }
        $ports += @{ Serial=$serial; Role=$spec.Role; Pending=''; Command=$spec.Command }
    }
    foreach ($entry in $ports) {
        $entry.Serial.DiscardInBuffer()
        if ($entry.Command) { $entry.Serial.WriteLine($entry.Command) }
    }
    $watch=[Diagnostics.Stopwatch]::StartNew()
    while ($watch.Elapsed.TotalSeconds -lt $Seconds) {
        foreach ($entry in $ports) {
            $entry.Pending += $entry.Serial.ReadExisting()
            while (($newline=$entry.Pending.IndexOf("`n")) -ge 0) {
                $line=$entry.Pending.Substring(0,$newline).TrimEnd("`r")
                $entry.Pending=$entry.Pending.Substring($newline+1)
                if ($line -match 'SP OOB|SP PEER') { throw 'Unexpected secret-bearing output; capture stopped without logging it.' }
                if ($line) { '[{0} {1:N3}s] {2}' -f $entry.Role,$watch.Elapsed.TotalSeconds,$line }
            }
        }
        Start-Sleep -Milliseconds 15
    }
} finally {
    foreach ($entry in $ports) { $entry.Serial.Close(); $entry.Serial.Dispose() }
}
