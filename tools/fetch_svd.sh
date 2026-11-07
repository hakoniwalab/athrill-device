#!/bin/sh
# Download the pinned CMSIS-SVD files listed in spec/svd-sources.yaml into $1 (default build/svd)
# and verify their SHA-256. The files are STMicroelectronics', Apache-2.0; they are not committed.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
out=${1:-$root/build/svd}
mkdir -p "$out"
python3 - "$root/spec/svd-sources.yaml" "$out" <<'PY'
import hashlib, sys, urllib.request, yaml, pathlib
src = yaml.safe_load(open(sys.argv[1]))["svd"]
out = pathlib.Path(sys.argv[2])
for soc, s in src.items():
    dest = out / s["file"]
    if not dest.exists() or hashlib.sha256(dest.read_bytes()).hexdigest() != s["sha256"]:
        repo = s["repository"].replace("https://github.com/", "")
        url = f"https://raw.githubusercontent.com/{repo}/{s['revision']}/{s['path']}"
        urllib.request.urlretrieve(url, dest)
    digest = hashlib.sha256(dest.read_bytes()).hexdigest()
    if digest != s["sha256"]:
        sys.exit(f"{dest}: sha256 {digest} != pinned {s['sha256']}")
    print(f"{soc}: {dest} ok")
PY
