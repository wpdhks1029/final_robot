#!/usr/bin/env python3
"""Prepare or copy a hash-verified gate bundle to a NEW leader directory only.

No robot process, lifecycle transition, motion command or arm request is made.
Default is plan-only. --apply transfers files using the existing SSH account.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

FILES=('nav2_convoy_gate.py','nav2_convoy_gate_core.py','leader_nav2_convoy.launch.py',
       'nav2_convoy_launch_core.py','convoy_source_time.py','start_leader_nav2.bash')


def build(root):
    source=root/'candidate/follower_diagnostics'
    manifest={name:hashlib.sha256((source/name).read_bytes()).hexdigest() for name in FILES}
    path=root/'leader_stability_candidate.tar.gz'
    # Deterministic metadata makes the destination revision repeatable.
    with tarfile.open(path,'w:gz') as archive:
        for name in (*FILES,'manifest.json'):
            data=((source/name).read_bytes() if name!='manifest.json' else
                  (json.dumps(manifest,sort_keys=True,indent=2)+'\n').encode())
            info=tarfile.TarInfo(name);info.size=len(data);info.mode=0o644;info.mtime=0
            archive.addfile(info,io.BytesIO(data))
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    # Gzip header time can differ across builds; content revision derives from manifest.
    revision=hashlib.sha256(json.dumps(manifest,sort_keys=True).encode()).hexdigest()[:12]
    return path,digest,'nav2_follow_stability_'+revision


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--apply',action='store_true')
    args=parser.parse_args();root=Path(__file__).resolve().parent
    path,digest,directory=build(root)
    print('LEADER_DIR=~/leader_diagnostics/'+directory,flush=True)
    if not args.apply:
        print('PLAN ONLY: no network connection or deployment.');return
    remote='''set -eu
mkdir -p "$HOME/leader_diagnostics"
stage=$(mktemp -d "$HOME/leader_diagnostics/.stability_stage.XXXXXXXX")
cleanup() { if test -d "$stage"; then rm -f "$stage"/*.py "$stage"/*.bash "$stage"/*.json "$stage"/bundle.tar.gz; rmdir "$stage"; fi; }
trap cleanup EXIT
cat > "$stage/bundle.tar.gz"
printf '%s  %s\\n' DIGEST "$stage/bundle.tar.gz" | sha256sum -c -
tar -xzf "$stage/bundle.tar.gz" -C "$stage"
rm "$stage/bundle.tar.gz"
dest="$HOME/leader_diagnostics/DIRECTORY"
if test -e "$dest"; then
  for name in FILES manifest.json; do cmp -s "$stage/$name" "$dest/$name"; done
  echo "ALREADY_VERIFIED=$dest"
else
  mv "$stage" "$dest"
  echo "DEPLOYED=$dest"
fi
'''.replace('DIGEST',digest).replace('DIRECTORY',directory).replace('FILES',' '.join(FILES))
    subprocess.run(['ssh','sunuk@10.253.245.81',remote],input=path.read_bytes(),check=True)


if __name__=='__main__':main()
