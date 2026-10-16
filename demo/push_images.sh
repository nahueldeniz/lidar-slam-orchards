#!/usr/bin/env bash
# Publish the benchmark images to the GitHub container registry (maintainers only; run once
# per release, from the repository root, on the machine that holds the images).
#
#   echo "$GHCR_TOKEN" | docker login ghcr.io -u nahueldeniz --password-stdin
#   ./demo/push_images.sh            # every image in demo/images.conf
#   ./demo/push_images.sh lego dlo   # only these keys
#
# The token needs the write:packages scope. For each image in demo/images.conf the script
#   1. tags the local build tag with the registry name if that tag does not exist yet;
#   2. for method images, refuses to push unless the image ID equals the one recorded for
#      the paper's campaign in env/images.json (FORCE=1 overrides);
#   3. pushes it and appends the registry digest to demo/images.digests.
# New packages on ghcr.io are private: afterwards make each one public (GitHub, Packages,
# Package settings, Change visibility) and connect it to the repository.
set -euo pipefail
cd "$(dirname "$0")/.."
CONF=demo/images.conf
want=("$@")
digests=demo/images.digests
expected_id() {   # campaign image ID of a local build tag (env/images.json), or empty
  python3 -c 'import json,sys; print(json.load(open("env/images.json")).get(sys.argv[1], {}).get("id", ""))' "$1"
}
while read -r key reg local _; do
  [[ -z "${key:-}" || "$key" == \#* ]] && continue
  if [ "${#want[@]}" -gt 0 ] && [[ ! " ${want[*]} " == *" $key "* ]]; then continue; fi
  if ! docker image inspect "$reg" >/dev/null 2>&1; then
    docker image inspect "$local" >/dev/null 2>&1 || { echo "ERROR: neither $reg nor $local exists" >&2; exit 1; }
    docker tag "$local" "$reg"
  fi
  id=$(docker image inspect --format '{{.Id}}' "$reg")
  exp=$(expected_id "$local")
  if [ -n "$exp" ] && [ "$id" != "$exp" ] && [ -z "${FORCE:-}" ]; then
    echo "ERROR: $reg is $id, but the campaign used $exp (env/images.json); not pushed (FORCE=1 to override)" >&2
    exit 1
  fi
  echo "== $key: pushing $reg ($id)"
  docker push "$reg"
  dig=$(docker image inspect --format '{{range .RepoDigests}}{{println .}}{{end}}' "$reg" | grep "^${reg%:*}@" | head -n 1)
  printf '%s\t%s\t%s\t%s\n' "$key" "$reg" "$dig" "$id" >> "$digests"
done < "$CONF"
echo "Pushed. Digests appended to $digests (commit it). Now make the packages public on GitHub."
