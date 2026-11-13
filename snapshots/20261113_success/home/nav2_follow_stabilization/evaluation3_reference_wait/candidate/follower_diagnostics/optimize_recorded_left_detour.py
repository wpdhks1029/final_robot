"""기존 기록 조건을 유지한 시간 비용 경로 재계산. 모터/ROS 접근 없음."""
import json
from pathlib import Path
from temporary_detour_candidate import Settings,plan


def main():
    logs=Path(__file__).parent/'logs';base=json.loads((logs/'recorded_left_detour_plan.json').read_text())
    s=base['snapshot']
    result=plan(base['points'],Settings(),side=1,initial_cross=s['cross_error_m'],
                initial_yaw=s['pose']['yaw']-s['reference_heading'],objective='time',nominal_speed=.06)
    if not result['path']:raise RuntimeError(result['reason'])
    base.update(result);dest=logs/'recorded_left_detour_time_plan.json'
    dest.write_text(json.dumps(base,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='path'},ensure_ascii=False),flush=True)
    print('JSON='+str(dest),flush=True)


if __name__=='__main__':main()
