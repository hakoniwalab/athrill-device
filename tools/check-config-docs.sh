#!/bin/sh
set -eu

repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
doc="$repo/docs/configuration.md"
tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT HUP INT TERM

find "$repo/board/common/src" -type f \( -name '*.c' -o -name '*.cpp' -o -name '*.h' \) -exec grep -hE \
  'get_devcfg_(value|value_hex|string)|get_number\(ops,[[:space:]]*"|snprintf\([^;]*"(DEVICE|DEBUG|HAKO)' {} + |
sed -n \
  -e 's/.*get_devcfg_[a-z_]*( *"\([A-Z][A-Z0-9_]*\)".*/\1/p' \
  -e 's/.*get_number(ops,[[:space:]]*"\([A-Z][A-Z0-9_]*\)".*/\1/p' \
  -e 's/.*snprintf([^;]*"\([A-Z][A-Z0-9_]*%[0-9]*[diu][A-Z0-9_]*\)".*/\1/p' |
sed 's/%[0-9]*[diu]/<n>/g' | sort -u > "$tmp"

missing=0
while IFS= read -r key; do
  if ! grep -Fq "$key" "$doc"; then
    echo "missing configuration key in docs/configuration.md: $key" >&2
    missing=1
  fi
done < "$tmp"

if [ "$missing" -ne 0 ]; then
  exit 1
fi

echo "configuration documentation covers $(wc -l < "$tmp" | tr -d ' ') keys"
