#!/usr/bin/env bash
# Package ChipCutter as a macOS Universal2 (arm64 + x86_64) .app bundle,
# compressed UDZO drag-to-Applications .dmg disk image, and companion CLI tarball.
#
# Usage:
#   scripts/package_macos.sh <version> [--out-dir dist] [--target-dir target]
#
# Optional environment variables for Apple Developer ID signing & notarization:
#   MACOS_SIGN_IDENTITY          Developer ID Application signing identity
#   APPLE_ID                     Apple ID email for notarytool submission
#   APPLE_TEAM_ID                10-character Apple Developer Team ID
#   APPLE_APP_SPECIFIC_PASSWORD  App-specific password for notarytool
#
# When MACOS_SIGN_IDENTITY is unset, the script applies ad-hoc signing
# (`codesign --force --deep --sign -`), which is required on Apple Silicon.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/package_macos.sh <version> [--out-dir <dir>] [--target-dir <dir>]

Options:
  --out-dir <dir>     Output directory for ChipCutter.app, .dmg, and .tar.gz (default: dist)
  --target-dir <dir>  Cargo target directory containing aarch64/x86_64 release builds (default: target)
  -h, --help          Show this help message
EOF
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 1
fi

VERSION=""
OUT_DIR="dist"
TARGET_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --out-dir)
      if [[ $# -lt 2 ]]; then
        echo "error: --out-dir requires an argument" >&2
        exit 1
      fi
      OUT_DIR="$2"
      shift 2
      ;;
    --out-dir=*)
      OUT_DIR="${1#*=}"
      shift
      ;;
    --target-dir)
      if [[ $# -lt 2 ]]; then
        echo "error: --target-dir requires an argument" >&2
        exit 1
      fi
      TARGET_DIR="$2"
      shift 2
      ;;
    --target-dir=*)
      TARGET_DIR="${1#*=}"
      shift
      ;;
    -*)
      echo "error: unknown option '$1'" >&2
      usage >&2
      exit 1
      ;;
    *)
      if [[ -z "${VERSION}" ]]; then
        VERSION="$1"
        shift
      else
        echo "error: unexpected positional argument '$1'" >&2
        usage >&2
        exit 1
      fi
      ;;
  esac
done

VERSION="${VERSION#v}"
if [[ -z "${VERSION}" ]]; then
  echo "error: version must be non-empty" >&2
  usage >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${TARGET_DIR}" ]]; then
  TARGET_DIR="${REPO_ROOT}/target"
fi

if [[ "${OUT_DIR}" != /* ]]; then
  OUT_DIR="${REPO_ROOT}/${OUT_DIR}"
fi

ARM64_GUI="${TARGET_DIR}/aarch64-apple-darwin/release/chip-gui"
ARM64_CLI="${TARGET_DIR}/aarch64-apple-darwin/release/chipcutter"
X86_64_GUI="${TARGET_DIR}/x86_64-apple-darwin/release/chip-gui"
X86_64_CLI="${TARGET_DIR}/x86_64-apple-darwin/release/chipcutter"

ICON_64="${REPO_ROOT}/packaging/icons/hicolor/64x64/apps/chipcutter.png"
ICON_128="${REPO_ROOT}/packaging/icons/hicolor/128x128/apps/chipcutter.png"
ICON_256="${REPO_ROOT}/packaging/icons/hicolor/256x256/apps/chipcutter.png"
ICON_512="${REPO_ROOT}/packaging/icons/hicolor/512x512/apps/chipcutter.png"
PLIST_TEMPLATE="${REPO_ROOT}/packaging/macos/Info.plist.in"

for required_file in \
  "${ARM64_GUI}" "${ARM64_CLI}" \
  "${X86_64_GUI}" "${X86_64_CLI}" \
  "${ICON_64}" "${ICON_128}" "${ICON_256}" "${ICON_512}" \
  "${PLIST_TEMPLATE}" \
  "${REPO_ROOT}/LICENSE-MIT" \
  "${REPO_ROOT}/LICENSE-APACHE" \
  "${REPO_ROOT}/README.md"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "error: required input file missing: ${required_file}" >&2
    exit 1
  fi
done

for tool in sips iconutil lipo plutil codesign ditto hdiutil tar; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "error: required macOS packaging tool '${tool}' not found in PATH" >&2
    exit 1
  fi
done

TMP_WORK="$(mktemp -d "${TMPDIR:-/tmp}/chipcutter-macos-pkg.XXXXXX")"
cleanup() {
  rm -rf "${TMP_WORK}"
}
trap cleanup EXIT

mkdir -p "${OUT_DIR}"

APP_BUNDLE="${OUT_DIR}/ChipCutter.app"
rm -rf "${APP_BUNDLE}"
mkdir -p "${APP_BUNDLE}/Contents/MacOS" "${APP_BUNDLE}/Contents/Resources"

echo "==> Building ChipCutter.icns from hicolor PNG icons..."
ICONSET_DIR="${TMP_WORK}/ChipCutter.iconset"
mkdir -p "${ICONSET_DIR}"

sips -z 16 16     "${ICON_64}"  --out "${ICONSET_DIR}/icon_16x16.png"      >/dev/null
sips -z 32 32     "${ICON_64}"  --out "${ICONSET_DIR}/icon_16x16@2x.png"   >/dev/null
sips -z 32 32     "${ICON_64}"  --out "${ICONSET_DIR}/icon_32x32.png"      >/dev/null
sips -z 64 64     "${ICON_64}"  --out "${ICONSET_DIR}/icon_32x32@2x.png"   >/dev/null
sips -z 128 128   "${ICON_128}" --out "${ICONSET_DIR}/icon_128x128.png"    >/dev/null
sips -z 256 256   "${ICON_256}" --out "${ICONSET_DIR}/icon_128x128@2x.png" >/dev/null
sips -z 256 256   "${ICON_256}" --out "${ICONSET_DIR}/icon_256x256.png"    >/dev/null
sips -z 512 512   "${ICON_512}" --out "${ICONSET_DIR}/icon_256x256@2x.png" >/dev/null
sips -z 512 512   "${ICON_512}" --out "${ICONSET_DIR}/icon_512x512.png"    >/dev/null
sips -z 1024 1024 "${ICON_512}" --out "${ICONSET_DIR}/icon_512x512@2x.png" >/dev/null

iconutil -c icns "${ICONSET_DIR}" -o "${APP_BUNDLE}/Contents/Resources/ChipCutter.icns"

echo "==> Combining arm64 + x86_64 binaries into Universal2 Mach-O executables..."
lipo -create "${ARM64_GUI}" "${X86_64_GUI}" -output "${APP_BUNDLE}/Contents/MacOS/chip-gui"
lipo -create "${ARM64_CLI}" "${X86_64_CLI}" -output "${APP_BUNDLE}/Contents/MacOS/chipcutter"
chmod 755 "${APP_BUNDLE}/Contents/MacOS/chip-gui" "${APP_BUNDLE}/Contents/MacOS/chipcutter"

echo "==> Rendering Info.plist (version ${VERSION}) and copying license files..."
sed "s|@VERSION@|${VERSION}|g" "${PLIST_TEMPLATE}" > "${APP_BUNDLE}/Contents/Info.plist"
plutil -lint "${APP_BUNDLE}/Contents/Info.plist"
cp "${REPO_ROOT}/LICENSE-MIT" "${REPO_ROOT}/LICENSE-APACHE" "${APP_BUNDLE}/Contents/Resources/"

if [[ -n "${MACOS_SIGN_IDENTITY:-}" ]]; then
  echo "==> Signing ChipCutter.app with Developer ID identity..."
  codesign --force --options runtime --timestamp --sign "${MACOS_SIGN_IDENTITY}" \
    "${APP_BUNDLE}/Contents/MacOS/chipcutter"
  codesign --force --options runtime --timestamp --sign "${MACOS_SIGN_IDENTITY}" \
    "${APP_BUNDLE}/Contents/MacOS/chip-gui"
  codesign --force --options runtime --timestamp --sign "${MACOS_SIGN_IDENTITY}" \
    "${APP_BUNDLE}"
else
  echo "==> Applying ad-hoc signature to ChipCutter.app (Apple Silicon compatible)..."
  codesign --force --deep --sign - "${APP_BUNDLE}"
fi

echo "==> Verifying code signature on ChipCutter.app..."
codesign --verify --deep --strict --verbose=2 "${APP_BUNDLE}"

DMG_PATH="${OUT_DIR}/ChipCutter-${VERSION}-macos-universal.dmg"
DMG_STAGE="${TMP_WORK}/dmg-stage"
mkdir -p "${DMG_STAGE}"

echo "==> Staging drag-to-Applications DMG image..."
ditto "${APP_BUNDLE}" "${DMG_STAGE}/ChipCutter.app"
ln -s /Applications "${DMG_STAGE}/Applications"
cp "${REPO_ROOT}/LICENSE-MIT" "${REPO_ROOT}/LICENSE-APACHE" "${DMG_STAGE}/"

rm -f "${DMG_PATH}"
hdiutil create \
  -volname "ChipCutter ${VERSION}" \
  -srcfolder "${DMG_STAGE}" \
  -ov \
  -format UDZO \
  "${DMG_PATH}"

if [[ -n "${MACOS_SIGN_IDENTITY:-}" && -n "${APPLE_ID:-}" && -n "${APPLE_TEAM_ID:-}" && -n "${APPLE_APP_SPECIFIC_PASSWORD:-}" ]]; then
  echo "==> Signing, notarizing, and stapling DMG..."
  codesign --force --timestamp --sign "${MACOS_SIGN_IDENTITY}" "${DMG_PATH}"
  xcrun notarytool submit "${DMG_PATH}" \
    --apple-id "${APPLE_ID}" \
    --team-id "${APPLE_TEAM_ID}" \
    --password "${APPLE_APP_SPECIFIC_PASSWORD}" \
    --wait
  xcrun stapler staple "${DMG_PATH}"
  xcrun stapler validate "${DMG_PATH}"
elif [[ -n "${MACOS_SIGN_IDENTITY:-}" ]]; then
  echo "==> Signing DMG (notarization credentials not set; skipping notarytool)..."
  codesign --force --timestamp --sign "${MACOS_SIGN_IDENTITY}" "${DMG_PATH}"
fi

CLI_ARCHIVE_NAME="chipcutter-${VERSION}-macos-universal"
CLI_STAGE="${TMP_WORK}/${CLI_ARCHIVE_NAME}"
CLI_TARBALL="${OUT_DIR}/${CLI_ARCHIVE_NAME}.tar.gz"

echo "==> Packaging companion Universal2 CLI archive..."
mkdir -p "${CLI_STAGE}"
cp "${APP_BUNDLE}/Contents/MacOS/chipcutter" "${CLI_STAGE}/chipcutter"
chmod 755 "${CLI_STAGE}/chipcutter"
cp "${REPO_ROOT}/LICENSE-MIT" "${REPO_ROOT}/LICENSE-APACHE" "${REPO_ROOT}/README.md" "${CLI_STAGE}/"
rm -f "${CLI_TARBALL}"
tar -czf "${CLI_TARBALL}" -C "${TMP_WORK}" "${CLI_ARCHIVE_NAME}"

echo "==> Packaged macOS artifacts:"
echo "  App bundle: ${APP_BUNDLE}"
echo "  Disk image: ${DMG_PATH}"
echo "  CLI bundle: ${CLI_TARBALL}"
