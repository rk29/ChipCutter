#!/usr/bin/env bash
set -euo pipefail

APP_ID="io.github.rk29.ChipCutter"
RUNTIME_REPO="https://flathub.org/repo/flathub.flatpakrepo"
BRANCH="stable"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MANIFEST="${REPO_ROOT}/packaging/flatpak/${APP_ID}.yml"

VERSION=""
OUT_DIR="${REPO_ROOT}/dist"
WORK_DIR="${REPO_ROOT}/target/flatpak"
ARCH="$(uname -m)"
INSTALL_USER=false

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Build a single-file Linux Flatpak bundle for ChipCutter (${APP_ID}).

Options:
  --version <ver>    Release version string (default: workspace.package.version from Cargo.toml)
  --out-dir <dir>    Directory for the output .flatpak bundle (default: dist)
  --work-dir <dir>   Scratch directory for flatpak-builder state, build, and OSTree repo (default: target/flatpak)
  --arch <arch>      Target Flatpak architecture (default: \$(uname -m), e.g. x86_64)
  --branch <branch>  OSTree export branch name (default: stable)
  --install-user     Install the produced bundle into the current user's Flatpak installation
  -h, --help         Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      [[ $# -ge 2 ]] || { echo "Error: --version requires an argument" >&2; exit 2; }
      VERSION="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || { echo "Error: --out-dir requires an argument" >&2; exit 2; }
      OUT_DIR="$2"
      shift 2
      ;;
    --work-dir)
      [[ $# -ge 2 ]] || { echo "Error: --work-dir requires an argument" >&2; exit 2; }
      WORK_DIR="$2"
      shift 2
      ;;
    --arch)
      [[ $# -ge 2 ]] || { echo "Error: --arch requires an argument" >&2; exit 2; }
      ARCH="$2"
      shift 2
      ;;
    --branch)
      [[ $# -ge 2 ]] || { echo "Error: --branch requires an argument" >&2; exit 2; }
      BRANCH="$2"
      shift 2
      ;;
    --install-user)
      INSTALL_USER=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: unknown argument '$1'" >&2
      usage >&2
      exit 2
      ;;
  esac
done

for tool in flatpak flatpak-builder; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Error: required tool '${tool}' not found in PATH" >&2
    exit 1
  fi
done

if [[ ! -f "${MANIFEST}" ]]; then
  echo "Error: Flatpak manifest not found at '${MANIFEST}'" >&2
  exit 1
fi

if [[ -z "${VERSION}" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    VERSION="$(python3 - "${REPO_ROOT}/Cargo.toml" <<'PY'
import sys, tomllib
with open(sys.argv[1], "rb") as f:
    data = tomllib.load(f)
print(data["workspace"]["package"]["version"])
PY
)"
  else
    VERSION="$(sed -n '/^\[workspace\.package\]/,/^\[/ s/^version = "\([^"]*\)"/\1/p' "${REPO_ROOT}/Cargo.toml" | head -n 1)"
  fi
fi

VERSION="${VERSION#v}"
if [[ -z "${VERSION}" ]]; then
  echo "Error: could not determine ChipCutter version from Cargo.toml; pass --version <ver>" >&2
  exit 1
fi
if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$ ]]; then
  echo "Error: invalid semantic version '${VERSION}'" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}" "${WORK_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd)"
WORK_DIR="$(cd "${WORK_DIR}" && pwd)"

BUILD_DIR="${WORK_DIR}/build-dir"
STATE_DIR="${WORK_DIR}/state"
OSTREE_REPO="${WORK_DIR}/repo"
BUNDLE_PATH="${OUT_DIR}/ChipCutter-${VERSION}-linux-${ARCH}.flatpak"

echo "==> Building ${APP_ID} (version=${VERSION}, arch=${ARCH}, branch=${BRANCH})"
flatpak-builder \
  --user \
  --force-clean \
  --arch="${ARCH}" \
  --state-dir="${STATE_DIR}" \
  --default-branch="${BRANCH}" \
  "${BUILD_DIR}" \
  "${MANIFEST}"

echo "==> Exporting OSTree repository to ${OSTREE_REPO}"
rm -rf "${OSTREE_REPO}"
flatpak build-export \
  --arch="${ARCH}" \
  "${OSTREE_REPO}" \
  "${BUILD_DIR}" \
  "${BRANCH}"

echo "==> Creating single-file bundle ${BUNDLE_PATH}"
flatpak build-bundle \
  --arch="${ARCH}" \
  --runtime-repo="${RUNTIME_REPO}" \
  "${OSTREE_REPO}" \
  "${BUNDLE_PATH}" \
  "${APP_ID}" \
  "${BRANCH}"

if [[ "${INSTALL_USER}" == "true" ]]; then
  echo "==> Installing ${BUNDLE_PATH} into user Flatpak installation"
  flatpak install --user --noninteractive --reinstall "${BUNDLE_PATH}"
fi

echo "==> Flatpak bundle ready: ${BUNDLE_PATH}"
ls -lh "${BUNDLE_PATH}"
