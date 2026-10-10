param([Parameter(Mandatory)][string]$Path)
$ErrorActionPreference='Stop'
$upper=[uint32]0
$ranges=[Collections.Generic.List[object]]::new()
$eof=$false
foreach($line in [IO.File]::ReadLines((Resolve-Path -LiteralPath $Path))) {
    if(!$line.StartsWith(':') -or (($line.Length-1)%2)) { throw 'Invalid Intel HEX line' }
    $bytes=[byte[]]::new(($line.Length-1)/2)
    for($i=0;$i -lt $bytes.Length;$i++) { $bytes[$i]=[Convert]::ToByte($line.Substring(1+2*$i,2),16) }
    if($bytes.Length -ne 5+$bytes[0]) { throw 'Invalid record length' }
    $checksum=0
    foreach($b in $bytes) { $checksum+=$b }
    if(($checksum -band 255) -ne 0) { throw 'Invalid checksum' }
    if($eof) { throw 'Record after EOF' }
    switch($bytes[3]) {
        0 {
            $start=$upper+256*$bytes[1]+$bytes[2]
            $end=$start+$bytes[0]
            # Preserve settings/NVS as well as UICR. The approved application
            # envelope ends before 0x174000, not at the end of physical RRAM.
            if($start -ge 0x174000 -or $end -gt 0x174000) { throw ('Refusing application-external/settings/UICR write at 0x{0:X8}' -f $start) }
            if($ranges.Count -and $ranges[$ranges.Count-1].End -eq $start) { $ranges[$ranges.Count-1].End=$end }
            else { $ranges.Add(@{Start=$start;End=$end}) }
        }
        1 { $eof=$true }
        2 { if($bytes[0] -ne 2) {throw 'Invalid ESA'}; $upper=[uint32](16*(256*$bytes[4]+$bytes[5])) }
        3 { if($bytes[0] -ne 4) {throw 'Invalid segment entry point record'} }
        4 { if($bytes[0] -ne 2) {throw 'Invalid ELA'}; $upper=[uint32](65536*(256*$bytes[4]+$bytes[5])) }
        5 { if($bytes[0] -ne 4) {throw 'Invalid entry point record'} }
        default { throw "Unsupported record type $($bytes[3])" }
    }
}
if(!$eof -or !$ranges.Count) { throw 'Incomplete or empty HEX' }
foreach($range in $ranges) { 'RRAM_SEGMENT 0x{0:X8}..0x{1:X8} bytes={2}' -f $range.Start,($range.End-1),($range.End-$range.Start) }
'HEX_CHECKSUM_RRAM_ONLY_PASS UICR_records=0 sha256='+ (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
