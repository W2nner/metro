param([string]$Data = "", [int]$Port = 8190)
Set-Location $PSScriptRoot
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
if (!$Data -and (Test-Path '../lidar-review')) { $Data = (Resolve-Path '../lidar-review').Path }
if ($Data) { python -m metro_detector serve --data $Data --port $Port --state "$PSScriptRoot/state" }
else { python -m metro_detector serve --port $Port --state "$PSScriptRoot/state" }
