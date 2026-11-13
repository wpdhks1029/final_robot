"""Compare complete candidate search, including static clearance selection."""
import json,sys,time
from pathlib import Path
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'candidate/follower_diagnostics'))
import continuous_avoidance_candidate as fast
import continuous_avoidance_reference as old
old.CLEARANCE=fast.CLEARANCE
from apf_pivot_candidate import obstacle_scene
rows=[]
for fixture in sorted((root/'fixtures').glob('*.json')):
 d=json.loads(fixture.read_text());scene=obstacle_scene(d['points'],d['start']);results={};elapsed={}
 for name,module in [('reference',old),('optimized',fast)]:
  begin=time.perf_counter();results[name]=module.search(scene['points'],d['start'],scene['box']);elapsed[name]=time.perf_counter()-begin
 assert results['reference']==results['optimized'],fixture.name
 result=results['optimized'];row=dict(fixture=fixture.name,full_result_identical=True,elapsed_s=elapsed,reason=result['reason'],selected={k:result.get(k) for k in ('offset_m','approach_m','rejoin_m','minimum_m')})
 rows.append(row);print(json.dumps(row),flush=True)
(root/'full_search_results.json').write_text(json.dumps(rows,indent=2)+'\n')
