#!/usr/bin/env python3
"""비무장·영속도와 빈 라이다 포트를 확인하고 8비트 고정 후보를 실행한다."""
import argparse,json,os,subprocess
from pathlib import Path
from run_lidar_sdk_trace import ROOT,ORIGINAL,DEVICE,stopped_status

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run:p.print_help();return
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    candidate=ROOT/'lidar_sdk_trace/build/lidar_sdk_trace_node'
    args=json.loads((ROOT/'logs/lidar_sdk_trace_restore_command.json').read_text())
    if not args or args[0]!=str(ORIGINAL):raise RuntimeError('보존된 원래 명령이 예상 실행파일과 다릅니다.')
    if not candidate.is_file():raise RuntimeError('후보 실행파일이 없습니다.')
    for i,value in enumerate(args[:-1]):
        if value=='--params-file' and not Path(args[i+1]).is_file():raise RuntimeError('원래 매개변수 파일이 없습니다.')
    stopped_status()
    result=subprocess.run(['fuser',DEVICE],capture_output=True)
    if result.returncode!=1:raise RuntimeError('라이다 포트가 사용 중이거나 확인 실패. 기존 프로세스를 종료하지 않았습니다.')
    print('비무장·영속도와 포트 해제 확인. 패킷 형식/길이 보호 및 강도8비트 고정 후보를 실행합니다. 이 터미널을 유지하세요.',flush=True)
    os.execv(str(candidate),[str(candidate),*args[1:],'--ros-args','-p','auto_intensity:=false'])

if __name__=='__main__':main()
