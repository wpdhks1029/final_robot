#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parent
rows=json.loads((root/'MANIFEST.json').read_text())['files']
bad=[r['path'] for r in rows if not (root/r['path']).is_file() or hashlib.sha256((root/r['path']).read_bytes()).hexdigest()!=r['sha256']]
print(json.dumps({'files':len(rows),'mismatches':bad},ensure_ascii=False))
sys.exit(bool(bad))
