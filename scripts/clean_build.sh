#!/usr/bin/env bash
set -Eeuo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

echo "This will remove:"
echo "  $WS/build"
echo "  $WS/install"
echo "  $WS/log"

read -r -p "Continue? Type YES: " answer

if [[ "$answer" != "YES" ]]; then
  echo "Cancelled."
  exit 0
fi

rm -rf -- "$WS/build" "$WS/install" "$WS/log"

echo "Build artifacts removed."
