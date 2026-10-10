# SPDX-License-Identifier: Apache-2.0
<#
Register the two explicitly identified NU54 boards through their wired USB VCOMs.
No firmware upload, reset, key export, bond deletion, or file/log output occurs.
OOB values exist only in process memory and the trusted USB transport. Managed
.NET strings cannot be reliably scrubbed; run in a short-lived PowerShell process
on a trusted PC. Do not attach a serial logger, debugger, or packet logger.
The remote boards must already run the matching secure_pair firmware.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][ValidatePattern('^COM[1-9][0-9]*$')][string]$DronePort,
    [Parameter(Mandatory)][ValidatePattern('^COM[1-9][0-9]*$')][string]$SearcherPort,
    [Parameter(Mandatory)][ValidatePattern('^[A-Fa-f0-9]{8,128}$')][string]$DroneUsbId,
    [Parameter(Mandatory)][ValidatePattern('^[A-Fa-f0-9]{8,128}$')][string]$SearcherUsbId,
    [ValidateRange(15, 100)][int]$TimeoutSeconds = 90,
    # Negative test only: corrupt both remote OOB confirms in RAM. Requires
    # both boards to have no owner/bond and never deletes existing credentials.
    [switch]$RejectInvalidOob
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$phase = 'USB identity check'
$droneSerial = $null
$searcherSerial = $null
$droneOob = $null
$searcherOob = $null
$complete = $false

function Assert-UsbIdentity {
    param([string]$PortName, [string]$ExpectedId)
    $serialDevices = @(Get-CimInstance Win32_SerialPort |
        Where-Object { $_.DeviceID -ieq $PortName })
    if ($serialDevices.Count -ne 1) {
        throw 'The requested COM port could not be uniquely identified.'
    }
    $instance = [string]$serialDevices[0].PNPDeviceID
    $matched = $false
    for ($depth = 0; $depth -lt 8 -and $instance; $depth++) {
        if (($instance -split '\\')[-1] -ieq $ExpectedId) {
            $matched = $true
            break
        }
        try {
            $parentProperty = Get-PnpDeviceProperty -InstanceId $instance `
                -KeyName 'DEVPKEY_Device_Parent' -ErrorAction Stop
            $instance = [string]$parentProperty.Data
        } catch {
            break
        }
    }
    if (-not $matched) {
        throw 'USB board identity does not exactly match the supplied role mapping. No enrollment performed.'
    }
}

function Open-EnrollmentPort {
    param([string]$PortName)
    $connection = [System.IO.Ports.SerialPort]::new($PortName, 115200,
        [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $connection.NewLine = "`n"
    $connection.ReadTimeout = 250
    $connection.WriteTimeout = 1500
    $connection.DtrEnable = $true
    $connection.RtsEnable = $false
    try {
        $connection.Open()
        return $connection
    } catch {
        $connection.Dispose()
        throw 'A USB port could not be opened. Close Arduino Monitor/Plotter and other serial readers.'
    }
}

function Get-SafeEnrollmentEvent {
    param([string]$Line)
    # Positive allowlist, never a general serial logger. Only fixed event names
    # and bounded numeric fields are reconstructed; commands/OOB/key data and
    # all unknown lines are discarded without printing any raw response.
    if ($Line -cmatch '^SP EVENT (ENROLLED|ENROLLMENT_REJECTED|PAIRING_FAILED|WINDOW_EXPIRED)$') {
        return ('event=' + $Matches[1])
    }
    if ($Line -cmatch '^SP ERR (LOCAL_UNAVAILABLE|OPEN_DENIED|PEER_REJECTED|ARM_DENIED|ERASE_FAILED|COMMAND|LINE_TOO_LONG)$') {
        return ('firmware_error=' + $Matches[1])
    }
    if ($Line -cmatch '^SP DIAG PAIR accept=([01]) state=([01]) features=([01]) oob=([01]) key128=([01]) sc=([01]) bond=([01])$') {
        return ('pair accept={0} state={1} features={2} oob={3} key128={4} sc={5} bond={6}' -f
            $Matches[1], $Matches[2], $Matches[3], $Matches[4], $Matches[5], $Matches[6], $Matches[7])
    }
    if ($Line -cmatch '^SP DIAG CONNECT accept=([01]) info=([01]) ready=([01]) fault=([01]) busy=([01]) owner=([01]) bond=([01]) window=([01]) armed=([01]) owner_match=([01]) enroll_match=([01])$') {
        return ('connect accept={0} info={1} ready={2} fault={3} busy={4} owner={5} bond={6} window={7} armed={8} owner_match={9} enroll_match={10}' -f
            $Matches[1], $Matches[2], $Matches[3], $Matches[4], $Matches[5], $Matches[6], $Matches[7], $Matches[8], $Matches[9], $Matches[10], $Matches[11])
    }
    if ($Line -cmatch '^SP DIAG OOB_CHECK accept=([01]) both=([01]) error=(-?[0-9]{1,6})$') {
        return ('oob_check accept={0} both={1} error={2}' -f $Matches[1], $Matches[2], $Matches[3])
    }
    if ($Line -cmatch '^SP DIAG FAILURE reason=([0-9]{1,3}) submitted=([01]) current=([01]) window=([01])$') {
        return ('auth_failure reason={0} submitted={1} current={2} window={3}' -f $Matches[1], $Matches[2], $Matches[3], $Matches[4])
    }
    if ($Line -cmatch '^SP DIAG SECURITY_REQUEST accepted=([01]) error=(-?[0-9]{1,6})$') {
        return ('security_request accepted={0} error={1}' -f $Matches[1], $Matches[2])
    }
    if ($Line -cmatch '^SP DIAG SECURITY level=([0-9]{1,3}) error=([0-9]{1,3})$') {
        return ('security_changed level={0} error={1}' -f $Matches[1], $Matches[2])
    }
    if ($Line -cmatch '^ADV (connect_start_error|connect_stop_error|independent_start_error)=(-?[0-9]{1,6})$') {
        return ('advertising_error={0} code={1}' -f $Matches[1], $Matches[2])
    }
    if ($Line -ceq 'LINK connected role=drone cs_authorized=0') {
        return 'link_connected role=drone awaiting_authorization=1'
    }
    if ($Line -cmatch '^LINK disconnected role=drone reason=([0-9]{1,3}) session_discarded=1 reboot=0$') {
        return ('link_disconnected role=drone reason=' + $Matches[1])
    }
    # Only this application's narrowly defined log messages are eligible.
    # Never strip a prefix and print the remainder of an arbitrary log line.
    if ($Line.Contains('nu54_searcher: ')) {
        $plain = $Line -creplace '\x1b\[[0-9;]*m', ''
        if ($plain -cmatch '^(?:\[[0-9:., ]{1,32}\]\s+)?<inf> nu54_searcher: CONNECTED epoch=([0-9]{1,10}) awaiting_L4_owner_authorization=1$') {
            return ('link_connected role=searcher epoch=' + $Matches[1])
        }
        if ($plain -cmatch '^(?:\[[0-9:., ]{1,32}\]\s+)?<inf> nu54_searcher: DISCONNECTED epoch=([0-9]{1,10}) reason=0x([a-fA-F0-9]{2}) session_discarded=1$') {
            return ('link_disconnected role=searcher epoch={0} reason=0x{1}' -f $Matches[1], $Matches[2])
        }
        if ($plain -cmatch '^(?:\[[0-9:., ]{1,32}\]\s+)?<wrn> nu54_searcher: (Scan start|Connect request) failed: (-?[0-9]{1,6})$') {
            return ('searcher_operation={0} code={1}' -f $Matches[1], $Matches[2])
        }
    }
    return $null
}

function Show-SafeEnrollmentState {
    param([string]$Role, $State)
    # Print only already-validated public status numbers, never the serial line.
    Write-Host ('ENROLL_STATE role={0} owner={1} bonds={2} window={3} armed={4} fault={5} authorized={6} pair_attempts={7} auth_failures={8} oob_rejections={9}' -f
        $Role, $State.Owner, $State.Bonds, $State.Window, $State.Armed, $State.Fault,
        $State.Authorized, $State.PairAttempts, $State.AuthFailures, $State.OobRejections)
}

function Invoke-SP {
    param([System.IO.Ports.SerialPort]$Connection, [string]$Command,
          [string]$ResponsePattern, [int]$LimitSeconds = 12)
    # DO NOT echo $Command or raw responses: either may contain OOB material.
    $Connection.WriteLine($Command)
    $deadline = [DateTime]::UtcNow.AddSeconds($LimitSeconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        $response = $null
        try {
            $response = $Connection.ReadLine().Trim()
        } catch [System.TimeoutException] {
            continue
        }
        if ($response.StartsWith('SP ERR ')) {
            $safeEvent = Get-SafeEnrollmentEvent $response
            if ($null -ne $safeEvent) { Write-Host ('ENROLL_EVENT port={0} {1}' -f $Connection.PortName, $safeEvent) }
            throw 'The firmware rejected the enrollment operation.'
        }
        if ($response -cmatch $ResponsePattern) {
            return $response
        }
        $safeEvent = Get-SafeEnrollmentEvent $response
        if ($null -ne $safeEvent) { Write-Host ('ENROLL_EVENT port={0} {1}' -f $Connection.PortName, $safeEvent) }
        # No raw serial response is printed or copied into a log.
        $response = $null
    }
    throw 'Timed out waiting for a valid secure enrollment response.'
}

function Get-SPStatus {
    param([System.IO.Ports.SerialPort]$Connection)
    $reply = Invoke-SP $Connection 'SP STATUS' `
        '^SP STATUS owner=[01] bonds=[0-9]+ window=[01] armed=[01] fault=[01] authorized=[01] pair_attempts=[0-9]+ auth_failures=[0-9]+ oob_rejections=[0-9]+$'
    if ($reply -cnotmatch '^SP STATUS owner=([01]) bonds=([0-9]+) window=([01]) armed=([01]) fault=([01]) authorized=([01]) pair_attempts=([0-9]+) auth_failures=([0-9]+) oob_rejections=([0-9]+)$') {
        throw 'Invalid status response.'
    }
    return [pscustomobject]@{
        Owner = [int]$Matches[1]
        Bonds = [int]$Matches[2]
        Window = [int]$Matches[3]
        Armed = [int]$Matches[4]
        Fault = [int]$Matches[5]
        Authorized = [int]$Matches[6]
        PairAttempts = [uint32]$Matches[7]
        AuthFailures = [uint32]$Matches[8]
        OobRejections = [uint32]$Matches[9]
    }
}

function Get-OobInMemory {
    param([System.IO.Ports.SerialPort]$Connection)
    $reply = Invoke-SP $Connection 'SP LOCAL' `
        '^SP OOB [01] [A-Fa-f0-9]{12} [A-Fa-f0-9]{32} [A-Fa-f0-9]{32}$'
    # Keep the format opaque to avoid unnecessary copies in diagnostic objects.
    return $reply.Substring(7)
}

function New-InvalidOobInMemory {
    param([string]$Oob)
    # The final character is the low nibble of the final confirm byte.
    $characters = $Oob.ToCharArray()
    try {
        $last = $characters.Length - 1
        $nibble = [Convert]::ToInt32([string]$characters[$last], 16)
        $characters[$last] = '0123456789abcdef'[($nibble -bxor 1)]
        return (-join $characters)
    } finally {
        [Array]::Clear($characters, 0, $characters.Length)
    }
}

function Test-EnrollmentComplete {
    param($DroneState, $SearcherState)
    # The caller first verified that both boards had no owner or bond. Firmware
    # saves owner only after L4 + SC + OOB + bonding checks have all succeeded.
    # A later CS setup failure may disconnect a correctly enrolled pair; current
    # Authorized therefore is deliberately not the registration success gate.
    return ($DroneState.Owner -eq 1 -and $SearcherState.Owner -eq 1 -and
        $DroneState.Bonds -eq 1 -and $SearcherState.Bonds -eq 1 -and
        $DroneState.Fault -eq 0 -and $SearcherState.Fault -eq 0)
}

try {
    if ($DronePort -ieq $SearcherPort -or $DroneUsbId -ieq $SearcherUsbId) {
        throw 'The drone and searcher must be different USB devices.'
    }
    Assert-UsbIdentity $DronePort $DroneUsbId
    Assert-UsbIdentity $SearcherPort $SearcherUsbId
    $phase = 'opening wired ports'
    $droneSerial = Open-EnrollmentPort $DronePort
    $searcherSerial = Open-EnrollmentPort $SearcherPort
    $droneSerial.DiscardInBuffer()
    $searcherSerial.DiscardInBuffer()

    $phase = 'checking existing ownership'
    $droneState = Get-SPStatus $droneSerial
    $searcherState = Get-SPStatus $searcherSerial
    $initialDroneState = $droneState
    $initialSearcherState = $searcherState
    foreach ($state in @($droneState, $searcherState)) {
        if ($state.Owner -or $state.Bonds -or $state.Fault) {
            throw 'Existing ownership, bonds, or a storage fault must be reviewed manually. This script never erases them.'
        }
    }

    $phase = 'opening explicit enrollment windows'
    [void](Invoke-SP $droneSerial 'SP CLOSE' '^SP OK CLOSE$')
    [void](Invoke-SP $searcherSerial 'SP CLOSE' '^SP OK CLOSE$')
    [void](Invoke-SP $droneSerial 'SP OPEN' '^SP OK OPEN$')
    [void](Invoke-SP $searcherSerial 'SP OPEN' '^SP OK OPEN$')

    $phase = 'exchanging one-time OOB data over USB'
    $droneOob = Get-OobInMemory $droneSerial
    $searcherOob = Get-OobInMemory $searcherSerial
    if ($RejectInvalidOob) {
        $droneOob = New-InvalidOobInMemory $droneOob
        $searcherOob = New-InvalidOobInMemory $searcherOob
    }
    [void](Invoke-SP $droneSerial ('SP PEER ' + $searcherOob) '^SP OK PEER$')
    [void](Invoke-SP $searcherSerial ('SP PEER ' + $droneOob) '^SP OK PEER$')
    # The data were never printed or saved. Drop managed references promptly.
    $droneOob = $null
    $searcherOob = $null

    $phase = 'arming both preselected peers'
    [void](Invoke-SP $droneSerial 'SP ARM' '^SP OK ARM$')
    [void](Invoke-SP $searcherSerial 'SP ARM' '^SP OK ARM$')
    if ($RejectInvalidOob) {
        Write-Host 'Negative test: modified OOB confirms exchanged in memory. Expecting explicit cryptographic rejection.'
    } else {
        Write-Host 'USB OOB exchange complete. Waiting for authenticated L4 pairing on both boards.'
    }

    $phase = 'verifying both L4 OOB enrollment transitions'
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $nextStatus = [DateTime]::UtcNow
    while ([DateTime]::UtcNow -lt $deadline) {
        $droneState = Get-SPStatus $droneSerial
        $searcherState = Get-SPStatus $searcherSerial
        if ([DateTime]::UtcNow -ge $nextStatus) {
            Show-SafeEnrollmentState 'drone' $droneState
            Show-SafeEnrollmentState 'searcher' $searcherState
            $nextStatus = [DateTime]::UtcNow.AddSeconds(5)
        }
        if ($RejectInvalidOob) {
            if ($droneState.Owner -or $searcherState.Owner -or $droneState.Bonds -or
                $searcherState.Bonds -or $droneState.Authorized -or $searcherState.Authorized) {
                $phase = 'SECURITY TEST FAILED: unexpected ownership, bond, or authorization'
                throw 'SECURITY TEST FAILED: modified OOB data resulted in a bond, owner, or authorized link.'
            }
            $attemptObserved = ($droneState.PairAttempts -gt $initialDroneState.PairAttempts) -and
                ($searcherState.PairAttempts -gt $initialSearcherState.PairAttempts)
            $rejectionObserved = ($droneState.OobRejections -gt $initialDroneState.OobRejections) -or
                ($searcherState.OobRejections -gt $initialSearcherState.OobRejections)
            if ($droneState.Fault -or $searcherState.Fault) {
                throw 'Negative test ended in a storage/ownership fault rather than clean rejection.'
            }
            if ($attemptObserved -and $rejectionObserved -and
                $droneState.Window -eq 0 -and $searcherState.Window -eq 0) {
                $complete = $true
                break
            }
            # No connection alone is NOT a passing negative test.
            Start-Sleep -Milliseconds 500
            continue
        }
        if ($droneState.Fault -or $searcherState.Fault) {
            throw 'A board entered fail-closed state during enrollment.'
        }
        if (Test-EnrollmentComplete $droneState $searcherState) {
            $complete = $true
            break
        }
        if (($droneState.Owner -eq 0 -and $droneState.Window -eq 0) -or
            ($searcherState.Owner -eq 0 -and $searcherState.Window -eq 0)) {
            throw 'An enrollment window ended without successful registration.'
        }
        Start-Sleep -Milliseconds 500
    }
    if (-not $complete) {
        throw 'The requested enrollment result was not confirmed on both boards within the allowed window.'
    }
    if ($RejectInvalidOob) {
        Write-Host 'PASS: an actual OOB pairing attempt was cryptographically rejected; neither board acquired an owner, bond, or authorized link.'
        Write-Host 'No credentials were erased. Run this script separately without -RejectInvalidOob for real registration.'
    } else {
        Write-Host 'SUCCESS: both boards completed L4 OOB enrollment and now report one registered owner, one bond, and no storage fault.'
        Write-Host 'Current connection security, Channel Sounding, and reboot/reconnect persistence require separate tests.'
        Write-Host 'No OOB values or Bluetooth keys were saved or displayed by this script.'
    }
} catch {
    # Do not surface $_, InvocationInfo, raw responses, or exception internals.
    Write-Error -Message (("Enrollment failed during {0}. No automatic key/bond deletion was performed. " +
        'Check SP STATUS on each board; an already enrolled side remains registered.') -f $phase) `
        -ErrorAction Continue
    exit 1
} finally {
    $droneOob = $null
    $searcherOob = $null
    foreach ($connection in @($droneSerial, $searcherSerial)) {
        if ($null -ne $connection) {
            try {
                if ($connection.IsOpen) {
                    # Close only the explicit enrollment window; never erase an owner.
                    $connection.WriteLine('SP CLOSE')
                    $connection.DiscardInBuffer()
                    $connection.Close()
                }
            } catch {
                # Closing errors must not leak previous raw serial data.
            } finally {
                $connection.Dispose()
            }
        }
    }
}
