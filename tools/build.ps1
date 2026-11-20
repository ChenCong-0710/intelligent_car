param(
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [switch]$Test,
    [switch]$Flash
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$bundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
function Find-BundleBinary([string]$Package,[string]$File) {
    $packageRoot = Join-Path $bundleRoot $Package
    if (Test-Path -LiteralPath $packageRoot) {
        $versions = Get-ChildItem -LiteralPath $packageRoot -Directory | Sort-Object { [version](($_.Name -split '\+')[0]) } -Descending
        foreach ($versionDir in $versions) {
            $candidate = Join-Path $versionDir.FullName "bin\$File"
            if (Test-Path -LiteralPath $candidate) { return $candidate }
        }
    }
    $command = Get-Command $File -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    throw "Cannot locate $File; install the $Package bundle in STM32Cube Bundles Manager."
}
$cmakeExe = Find-BundleBinary 'cmake' 'cmake.exe'
$ninjaExe = Find-BundleBinary 'ninja' 'ninja.exe'
$gccExe = Find-BundleBinary 'gnu-tools-for-stm32' 'arm-none-eabi-gcc.exe'
Push-Location -LiteralPath $projectRoot
try {
    if ($Test) {
        $hostGcc = (Get-Command gcc.exe -ErrorAction Stop).Source
        & $cmakeExe -S tests -B build/host -G Ninja "-DCMAKE_MAKE_PROGRAM=$ninjaExe" "-DCMAKE_C_COMPILER=$hostGcc" -DCMAKE_BUILD_TYPE=Debug
        if ($LASTEXITCODE -ne 0) { throw 'Host configure failed' }
        & $cmakeExe --build build/host
        if ($LASTEXITCODE -ne 0) { throw 'Host build failed' }
        $ctestExe = Join-Path (Split-Path $cmakeExe) 'ctest.exe'
        & $ctestExe --test-dir build/host --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    } else {
        & $cmakeExe --preset $Configuration "-DARM_GCC_BIN=$(Split-Path $gccExe)" "-DCMAKE_MAKE_PROGRAM=$ninjaExe"
        if ($LASTEXITCODE -ne 0) { throw 'Firmware configure failed' }
        & $cmakeExe --build --preset $Configuration
        if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed' }
        if ($Flash) {
            $programmer = Find-BundleBinary 'programmer' 'STM32_Programmer_CLI.exe'
            & $programmer -c port=SWD freq=1000 mode=UR reset=HWrst -w "build/$Configuration/line-following.elf" -v -rst
            if ($LASTEXITCODE -ne 0) { throw 'Flash/verify failed' }
        }
    }
} finally { Pop-Location }
