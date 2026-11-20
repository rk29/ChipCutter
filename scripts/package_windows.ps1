<#
.SYNOPSIS
    Packages ChipCutter Windows x86_64 release artifacts (.zip portable archive and .msi installer).

.DESCRIPTION
    Stages chip-gui.exe, chipcutter.exe, LICENSE-MIT, LICENSE-APACHE, and README.md,
    optionally signs executables and the MSI via signtool.exe when WINDOWS_SIGN_PFX_BASE64
    and WINDOWS_SIGN_PASSWORD are set, builds the portable ZIP archive, and compiles the
    WiX v5 MSI installer from packaging/windows/chipcutter.wxs.

.PARAMETER Version
    Release semantic version (e.g. "0.8.0" or "0.8.0-rc.1"). Defaults to [workspace.package].version in Cargo.toml.

.PARAMETER TargetDir
    Directory containing compiled chip-gui.exe and chipcutter.exe.
    Defaults to "target/x86_64-pc-windows-msvc/release".

.PARAMETER OutDir
    Output directory for release artifacts. Defaults to "dist".
#>
[CmdletBinding()]
param(
    [string]$Version = "",
    [string]$TargetDir = "target/x86_64-pc-windows-msvc/release",
    [string]$OutDir = "dist"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

function Resolve-RepoPath {
    param([Parameter(Mandatory = $true)][string]$PathValue)
    if ([System.IO.Path]::IsPathRooted($PathValue)) {
        return $PathValue
    }
    return Join-Path $RepoRoot $PathValue
}

if ([string]::IsNullOrWhiteSpace($Version)) {
    $cargoTomlPath = Join-Path $RepoRoot "Cargo.toml"
    $cargoContent = Get-Content -Path $cargoTomlPath -Raw
    if ($cargoContent -match '(?ms)\[workspace\.package\].*?^\s*version\s*=\s*"([^"]+)"') {
        $Version = $Matches[1]
    } else {
        throw "Could not determine workspace version from $cargoTomlPath; pass -Version explicitly."
    }
}

$Version = $Version.TrimStart('v')

# Windows Installer Package/@Version must be numeric Major.Minor.Build[.Revision] (no prerelease suffix).
$MsiVersion = ($Version -split '[-+]')[0]
if ($MsiVersion -notmatch '^\d+\.\d+\.\d+(\.\d+)?$') {
    throw "Invalid semantic version '$Version' (derived MSI version '$MsiVersion' is not numeric Major.Minor.Patch)."
}

$ResolvedTargetDir = Resolve-RepoPath $TargetDir
$ResolvedOutDir = Resolve-RepoPath $OutDir
$WxsPath = Resolve-RepoPath "packaging/windows/chipcutter.wxs"
$IconPath = Resolve-RepoPath "packaging/windows/chipcutter.ico"

$RequiredInputs = @(
    (Join-Path $ResolvedTargetDir "chip-gui.exe"),
    (Join-Path $ResolvedTargetDir "chipcutter.exe"),
    (Join-Path $RepoRoot "LICENSE-MIT"),
    (Join-Path $RepoRoot "LICENSE-APACHE"),
    (Join-Path $RepoRoot "README.md"),
    $WxsPath,
    $IconPath
)

foreach ($item in $RequiredInputs) {
    if (-not (Test-Path -LiteralPath $item -PathType Leaf)) {
        throw "Required packaging input not found: $item"
    }
}

New-Item -ItemType Directory -Path $ResolvedOutDir -Force | Out-Null

$StageDir = Join-Path $ResolvedOutDir ("stage-windows-x86_64-" + [guid]::NewGuid().ToString("N"))
$TempPfxPath = $null

function Find-SignTool {
    $cmd = Get-Command "signtool.exe" -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }
    $kitRoot = Join-Path ${env:ProgramFiles(x86)} "Windows Kits\10\bin"
    if (Test-Path -LiteralPath $kitRoot) {
        $candidate = Get-ChildItem -Path $kitRoot -Recurse -Filter "signtool.exe" -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -match '\\x64\\signtool\.exe$' } |
            Sort-Object FullName -Descending |
            Select-Object -First 1
        if ($candidate) {
            return $candidate.FullName
        }
    }
    throw "Authenticode signing requested, but signtool.exe was not found on PATH or in Windows Kits 10."
}

$SigningEnabled = (-not [string]::IsNullOrWhiteSpace($env:WINDOWS_SIGN_PFX_BASE64)) -and
                  (-not [string]::IsNullOrWhiteSpace($env:WINDOWS_SIGN_PASSWORD))

$SignToolPath = $null
if ($SigningEnabled) {
    $SignToolPath = Find-SignTool
    $TempPfxPath = Join-Path $env:TEMP ("chipcutter-sign-" + [guid]::NewGuid().ToString("N") + ".pfx")
    $pfxBytes = [System.Convert]::FromBase64String($env:WINDOWS_SIGN_PFX_BASE64)
    [System.IO.File]::WriteAllBytes($TempPfxPath, $pfxBytes)
    Write-Host "[package_windows] Authenticode signing enabled using $SignToolPath"
} else {
    Write-Host "[package_windows] Authenticode signing skipped (WINDOWS_SIGN_PFX_BASE64 / WINDOWS_SIGN_PASSWORD not set)."
}

function Invoke-AuthenticodeSign {
    param([Parameter(Mandatory = $true)][string]$FilePath)
    if (-not $SigningEnabled) {
        return
    }
    Write-Host "[package_windows] Signing $FilePath..."
    & $SignToolPath sign `
        /f $TempPfxPath `
        /p $env:WINDOWS_SIGN_PASSWORD `
        /fd SHA256 `
        /tr "http://timestamp.digicert.com" `
        /td SHA256 `
        $FilePath
    if ($LASTEXITCODE -ne 0) {
        throw "signtool.exe failed on '$FilePath' with exit code $LASTEXITCODE"
    }
}

try {
    New-Item -ItemType Directory -Path $StageDir -Force | Out-Null

    Write-Host "[package_windows] Staging release files into $StageDir..."
    Copy-Item -LiteralPath (Join-Path $ResolvedTargetDir "chip-gui.exe") -Destination $StageDir
    Copy-Item -LiteralPath (Join-Path $ResolvedTargetDir "chipcutter.exe") -Destination $StageDir
    Copy-Item -LiteralPath (Join-Path $RepoRoot "LICENSE-MIT") -Destination $StageDir
    Copy-Item -LiteralPath (Join-Path $RepoRoot "LICENSE-APACHE") -Destination $StageDir
    Copy-Item -LiteralPath (Join-Path $RepoRoot "README.md") -Destination $StageDir

    Invoke-AuthenticodeSign -FilePath (Join-Path $StageDir "chip-gui.exe")
    Invoke-AuthenticodeSign -FilePath (Join-Path $StageDir "chipcutter.exe")

    $ZipPath = Join-Path $ResolvedOutDir "ChipCutter-$Version-windows-x86_64.zip"
    if (Test-Path -LiteralPath $ZipPath) {
        Remove-Item -LiteralPath $ZipPath -Force
    }
    Write-Host "[package_windows] Creating portable archive $ZipPath..."
    Compress-Archive -Path (Join-Path $StageDir "*") -DestinationPath $ZipPath -CompressionLevel Optimal -Force

    $wixCmd = Get-Command "wix" -ErrorAction SilentlyContinue
    if (-not $wixCmd) {
        throw "WiX CLI ('wix') not found on PATH. Install via: dotnet tool install --global wix --version 5.*"
    }

    $MsiPath = Join-Path $ResolvedOutDir "ChipCutter-$Version-windows-x86_64.msi"
    if (Test-Path -LiteralPath $MsiPath) {
        Remove-Item -LiteralPath $MsiPath -Force
    }
    Write-Host "[package_windows] Building MSI installer $MsiPath (MSI Version=$MsiVersion)..."
    & $wixCmd.Source build `
        -arch x64 `
        -d "Version=$MsiVersion" `
        -d "StageDir=$StageDir" `
        -d "IconPath=$IconPath" `
        -o $MsiPath `
        $WxsPath
    if ($LASTEXITCODE -ne 0) {
        throw "wix build failed with exit code $LASTEXITCODE"
    }

    Invoke-AuthenticodeSign -FilePath $MsiPath

    $zipSizeMb = [math]::Round((Get-Item -LiteralPath $ZipPath).Length / 1MB, 2)
    $msiSizeMb = [math]::Round((Get-Item -LiteralPath $MsiPath).Length / 1MB, 2)
    Write-Host "[package_windows] Done!"
    Write-Host "  ZIP: $ZipPath ($zipSizeMb MB)"
    Write-Host "  MSI: $MsiPath ($msiSizeMb MB)"
} finally {
    if (Test-Path -LiteralPath $StageDir) {
        Remove-Item -LiteralPath $StageDir -Recurse -Force -ErrorAction SilentlyContinue
    }
    if ($TempPfxPath -and (Test-Path -LiteralPath $TempPfxPath)) {
        Remove-Item -LiteralPath $TempPfxPath -Force -ErrorAction SilentlyContinue
    }
}
