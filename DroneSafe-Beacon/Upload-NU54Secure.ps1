# Requires Windows PowerShell 5.1+ or PowerShell 7, installed NCS v3.4.1
# toolchain with pyOCD 0.42.0 + intelhex, and nrf_ocd 0.3.8.
# Close serial monitors/plotters and all other SWD tools before uploading.
param(
    [Parameter(Mandatory)][ValidateSet('drone','searcher')][string]$Role,
    [Parameter(Mandatory)][ValidatePattern('^[A-Fa-f0-9]{8,128}$')][string]$UsbId,
    [Parameter(Mandatory)][string]$HexPath,
    [Parameter(Mandatory)][string]$Programmer,
    [Parameter(Mandatory)][string]$SdkManager,
    [Parameter(Mandatory)][string]$SdkInstallRoot,
    [string]$LogDirectory,
    [switch]$ValidateOnly
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$taskUid = $UsbId.ToLowerInvariant()
# The caller maps the physical role to a complete probe UID; never select the
# first available probe. Both Python helpers independently enforce exact equality.
# Both bridges have produced transport read failures at 1 MHz / 1024 bytes.
# Keep the verified conservative settings; do not auto-retry a failed verify.
$taskFrequency = 100000
$taskControlFrequency = 1000000 # CoreSight discovery fails at 100 kHz on these bridges.
$taskChunk = 64
$taskSdkRoot = Join-Path $SdkInstallRoot 'v3.4.1'
if (!$LogDirectory) { $LogDirectory = Join-Path $PSScriptRoot '.build/upload-logs' }
$taskInspector = Join-Path $PSScriptRoot 'common/Inspect-Nu54Hex.ps1'
$taskReadback = Join-Path $PSScriptRoot 'common/Test-Nu54Readback.py'
$taskControl = Join-Path $PSScriptRoot 'common/Nu54-ProbeControl.py'
foreach ($taskRequired in @($HexPath,$Programmer,$SdkManager,$taskInspector,$taskReadback,$taskControl,(Join-Path $taskSdkRoot 'nrf/west.yml'))) {
    if (!(Test-Path -LiteralPath $taskRequired -PathType Leaf)) { throw "Required file is missing: $taskRequired" }
}
$taskHex = (Resolve-Path -LiteralPath $HexPath).Path
$Programmer = (Resolve-Path -LiteralPath $Programmer).Path
$SdkManager = (Resolve-Path -LiteralPath $SdkManager).Path
$SdkInstallRoot = (Resolve-Path -LiteralPath $SdkInstallRoot).Path
$taskSdkRoot = (Resolve-Path -LiteralPath $taskSdkRoot).Path
if ([IO.Path]::GetExtension($taskHex) -ine '.hex') { throw 'Only Intel HEX firmware is accepted.' }
$taskImageHash = (Get-FileHash -LiteralPath $taskHex -Algorithm SHA256).Hash
$null = New-Item -ItemType Directory -Path $LogDirectory -Force
$taskLog = Join-Path $LogDirectory ("upload-{0}-{1}-{2}.log" -f $Role,(Get-Date -Format 'yyyyMMdd-HHmmss-fff'),([guid]::NewGuid().ToString('N').Substring(0,8)))

function Write-TaskLog([string]$Message) {
    Add-Content -LiteralPath $taskLog -Value $Message -Encoding UTF8
    Write-Host $Message
}

function Invoke-CheckedNative {
    param([string]$Executable,[string[]]$Arguments,[string]$Expected)
    $taskCaptured = [Collections.Generic.List[string]]::new()
    $taskOldPreference = $ErrorActionPreference
    try {
        # Check native exit status and explicit marker, not PowerShell $? alone.
        $ErrorActionPreference = 'Continue'
        & $Executable @Arguments 2>&1 | ForEach-Object {
            $taskLine = $_.ToString()
            $taskCaptured.Add($taskLine)
            Write-TaskLog $taskLine
        }
        $taskExit = $LASTEXITCODE
    } finally { $ErrorActionPreference = $taskOldPreference }
    $taskText = $taskCaptured -join "`n"
    if ($taskExit -ne 0) { throw "Native command failed with exit code $taskExit; inspect $taskLog" }
    if ($taskText -match '(?im)\bERROR\b|Traceback|READBACK_FAILED|PROBE_CONTROL_FAILED') {
        throw "Command reported an error despite its exit code; inspect $taskLog"
    }
    if ($Expected -and $taskText -notmatch $Expected) { throw "Expected success marker missing; inspect $taskLog" }
}

$taskLaunch = @('toolchain','launch','--ncs-version','v3.4.1','--install-dir',$SdkInstallRoot,'--chdir',$taskSdkRoot,'--','python','-B')
$taskStage = 'offline_validation'
try {
    Write-TaskLog "UPLOAD_PLAN role=$Role exact_uid=$taskUid image_sha256=$taskImageHash swd_hz=$taskFrequency control_hz=$taskControlFrequency read_chunk=$taskChunk"
    & $taskInspector -Path $taskHex | ForEach-Object { Write-TaskLog $_ }
    Invoke-CheckedNative -Executable $SdkManager -Arguments ($taskLaunch + @($taskControl,'--role',$Role,'--uid',$taskUid,'--frequency',"$taskFrequency",'--check-dependencies')) -Expected '(?m)^DEPENDENCIES_VALID .*hardware_access=0$'
    Invoke-CheckedNative -Executable $SdkManager -Arguments ($taskLaunch + @($taskReadback,'--role',$Role,'--uid',$taskUid,'--hex',$taskHex,'--dry-run')) -Expected '(?m)^OFFLINE_PLAN_VALID hardware_access=0 verification_performed=0$'
    # --help does not enumerate probes; main prints the version before usage.
    # This release does not have a --version option.
    Invoke-CheckedNative -Executable $Programmer -Arguments @('--help') -Expected '\bnrf_ocd 0\.3\.8\b'
    if ($ValidateOnly) {
        Write-TaskLog 'OFFLINE_UPLOAD_VALID hardware_access=0 upload_performed=0'
        return
    }
    $taskStage = 'reset_halt_prepare'
    # nrf_ocd 0.3.8 reset-halt has a DHCSR/DFSR address bug. Use pyOCD's
    # system reset_and_halt, with state verification and no user scripts.
    Invoke-CheckedNative -Executable $SdkManager -Arguments ($taskLaunch + @($taskControl,'--role',$Role,'--uid',$taskUid,'--frequency',"$taskControlFrequency",'--mode','prepare')) -Expected "(?m)^RESET_HALT_CONFIRMED role=$Role state=HALTED$"
    if ((Get-FileHash -LiteralPath $taskHex -Algorithm SHA256).Hash -ne $taskImageHash) { throw 'Firmware changed during preflight. Stop the build before uploading.' }
    $taskStage = 'program_application_only'
    # No erase, auto-unlock, UICR/settings write, or reset/run here. The CPU
    # stays halted until the complete application image is read back.
    Invoke-CheckedNative -Executable $Programmer -Arguments @('-u',$taskUid,'-t','nrf54l15','-f',"$([int]($taskFrequency / 1000))",'-e','none','--no-verify','load',$taskHex) -Expected 'Upload complete'
    $taskStage = 'full_readback'
    Invoke-CheckedNative -Executable $SdkManager -Arguments ($taskLaunch + @($taskReadback,'--role',$Role,'--uid',$taskUid,'--hex',$taskHex,'--halt','--frequency',"$taskFrequency",'--chunk-bytes',"$taskChunk",'--max-seconds','180')) -Expected "(?m)^FULL_READBACK_MATCH role=$Role bytes=\d+ seconds=[\d.]+ full_verification=1$"
    if ((Get-FileHash -LiteralPath $taskHex -Algorithm SHA256).Hash -ne $taskImageHash) { throw 'Firmware changed during verification; reset/run withheld.' }
    $taskStage = 'start_verified_image'
    # Ordinary reset is separate from reset-halt. Confirm CPU state afterward;
    # a running CPU is not evidence of healthy IMU/BLE application operation.
    Invoke-CheckedNative -Executable $Programmer -Arguments @('-u',$taskUid,'-t','nrf54l15','-f',"$([int]($taskFrequency / 1000))",'reset') -Expected ''
    Invoke-CheckedNative -Executable $SdkManager -Arguments ($taskLaunch + @($taskControl,'--role',$Role,'--uid',$taskUid,'--frequency',"$taskControlFrequency",'--mode','check-running')) -Expected "(?m)^CPU_STARTED_CONFIRMED role=$Role state=(RUNNING|SLEEPING) boot_health_unchecked=1$"
    Write-TaskLog "UPLOAD_VERIFIED role=$Role application_full_readback=1 cpu_started=1 functional_test_performed=0"
    Write-TaskLog 'Check USB serial at 115200 for BOOT/STATUS. IMU, OLED, authentication and Channel Sounding are separate functional tests.'
} catch {
    Write-TaskLog "UPLOAD_FAILED stage=$taskStage success=0 message=$($_.Exception.Message)"
    Write-TaskLog 'No automatic erase/unlock/retry. The board may remain halted. Inspect this log before retrying.'
    throw
}
