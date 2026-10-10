# Build only; never enumerates, resets, or programs USB devices.
param(
    [Parameter(Mandatory)][ValidateSet('drone','searcher')][string]$Role,
    [Parameter(Mandatory)][string]$SdkInstallRoot,
    [Parameter(Mandatory)][string]$SdkManager,
    [string]$BuildRoot,
    [switch]$Pristine
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (!$BuildRoot) { $BuildRoot = Join-Path $PSScriptRoot '.build' }
$taskSdkRoot = Join-Path $SdkInstallRoot 'v3.4.1'
foreach ($taskRequired in @($SdkManager,(Join-Path $taskSdkRoot 'nrf/west.yml'))) {
    if (!(Test-Path -LiteralPath $taskRequired -PathType Leaf)) { throw "Required file is missing: $taskRequired" }
}
$taskSdkRoot = (Resolve-Path -LiteralPath $taskSdkRoot).Path
$SdkInstallRoot = (Resolve-Path -LiteralPath $SdkInstallRoot).Path
$SdkManager = (Resolve-Path -LiteralPath $SdkManager).Path
$taskSource = (Join-Path $PSScriptRoot $Role).Replace('\','/')
$taskBoardRoot = (Join-Path $PSScriptRoot 'board_root').Replace('\','/')
$taskBuild = [IO.Path]::GetFullPath((Join-Path $BuildRoot $Role)).Replace('\','/')
$taskMode = if ($Pristine) { 'always' } else { 'auto' }
# Always pass source/board explicitly: do not silently reuse another project's cache.
& $SdkManager toolchain launch --ncs-version v3.4.1 --install-dir $SdkInstallRoot --chdir $taskSdkRoot -- west build --sysbuild -p $taskMode -s $taskSource -d $taskBuild -b nrf54l15dk/nrf54l15/cpuapp/nu54dk -- "-DBOARD_ROOT=$taskBoardRoot"
if ($LASTEXITCODE -ne 0) { throw "Build failed for $Role. No board was programmed." }
$taskHex = "$taskBuild/$Role/zephyr/zephyr.hex"
if (!(Test-Path -LiteralPath $taskHex)) { throw "Build output HEX missing for $Role. No board was programmed." }
Write-Output "BUILD_COMPLETE role=$Role hex=$taskHex upload_performed=0"
