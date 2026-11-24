#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Rebuild the self-contained Nordic notification-completion extension at a pinned upstream version.
# Keep the modified dependency local and out of the repository.
mkdir -p .deps
module_dir="$(GOTOOLCHAIN=auto go mod download -json tinygo.org/x/bluetooth@v0.13.0 | python3 -c 'import json,sys; print(json.load(sys.stdin)["Dir"])')"
rm -rf .deps/bluetooth
cp -a "$module_dir" .deps/bluetooth
chmod -R u+w .deps/bluetooth
(cd .deps/bluetooth && patch --batch --forward -p1 < ../../patches/tinygo-bluetooth-notification-complete.patch)
go mod edit -replace=tinygo.org/x/bluetooth=./.deps/bluetooth
printf 'Pinned TinyGo Bluetooth 0.13.0 + notification completion patch ready\n'
