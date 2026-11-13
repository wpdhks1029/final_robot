#!/usr/bin/env python3
"""정지 상태에서 라이다만 진단 실행파일로 교체하고 원래 명령으로 복원한다."""
import argparse,json,os,signal,subprocess,sys,time
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parent
ORIGINAL=Path.home()/'robot_ws/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node'
DEVICE='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0'


def stopped_status():
    import rclpy
    from std_msgs.msg import String
    rclpy.init(args=[]);n=rclpy.create_node('lidar_sdk_trace_readiness');received=[]
    def cb(m):
        try:received.append((time.monotonic(),json.loads(m.data)))
        except ValueError:pass
    sub=n.create_subscription(String,'/follower/motor_candidate/status',cb,1)
    try:
        end=time.monotonic()+8
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.05)
            if received:
                rx,d=received[-1]
                if time.monotonic()-rx<=.3 and d.get('armed') is False and d.get('feedback_fresh') is True and d.get('measured_rpm')==[0,0] and d.get('sent_rpm')==[0,0]:return
        raise RuntimeError('최신 비무장·영속도 상태를 확인하지 못해 라이다를 종료하지 않았습니다.')
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');p.add_argument('--seconds',type=int,default=90)
    p.add_argument('--fixed-intensity',action='store_true',help='진단 중 강도8비트 고정. 원래 드라이버 복원 시 원래 설정 유지')
    a=p.parse_args()
    if not a.run:p.print_help();return
    if os.environ.get('ROS_DOMAIN_ID')!='10':p.error('ROS_DOMAIN_ID=10이 필요합니다.')
    if not 30<=a.seconds<=180:p.error('seconds는 30~180초입니다.')
    candidate=ROOT/'lidar_sdk_trace/build/lidar_sdk_trace_node'
    if not candidate.is_file():raise RuntimeError('진단 실행파일이 없습니다.')
    owners=[]
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:args=[os.fsdecode(x) for x in path.read_bytes().split(b'\0') if x]
        except (OSError,PermissionError):continue
        if args and args[0]==str(ORIGINAL):owners.append((int(path.parent.name),args))
    if len(owners)!=1:raise RuntimeError(f'기존 라이다 프로세스가 정확히 1개여야 합니다. 발견={len(owners)}. 종료하지 않았습니다.')
    pid,args=owners[0]
    # 시작 전에 기존 명령의 매개변수 파일도 복원 가능 여부를 확인한다.
    for i,arg in enumerate(args[:-1]):
        if arg=='--params-file' and not Path(args[i+1]).is_file():raise RuntimeError('복원에 필요한 매개변수 파일이 없습니다.')
    stopped_status()
    directory=ROOT/'logs';directory.mkdir(exist_ok=True);log=directory/f'lidar_sdk_trace_{time.time_ns()}.log'
    (directory/'lidar_sdk_trace_restore_command.json').write_text(json.dumps(args,ensure_ascii=False)+'\n')
    print(f'정지 상태 확인. 라이다 PID {pid}만 종료합니다. EKF·AMCL·모터 노드는 유지합니다.',flush=True)
    os.kill(pid,signal.SIGINT)
    child=None
    try:
        until=time.monotonic()+8
        while time.monotonic()<until:
            check=subprocess.run(['fuser',DEVICE],capture_output=True)
            if check.returncode==1:break
            if check.returncode!=0:raise RuntimeError('라이다 포트 소유 확인 실패')
            time.sleep(.1)
        else:raise RuntimeError('라이다 포트가 해제되지 않았습니다.')
        print(f'진단 {a.seconds}초. 주행하지 마세요. 종료 후 원래 라이다 명령을 자동 복원합니다.\nLOG={log}',flush=True)
        with log.open('x') as stream:
            diagnostic_args=[str(candidate),*args[1:]]
            if a.fixed_intensity:diagnostic_args+=['--ros-args','-p','auto_intensity:=false']
            child=subprocess.Popen(diagnostic_args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            try:child.wait(timeout=a.seconds)
            except subprocess.TimeoutExpired:pass
    except KeyboardInterrupt:
        print('진단 중단. 원래 라이다를 복원합니다.',flush=True)
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                print('진단 프로세스가 종료되지 않아 중복 실행을 막았습니다. 자동 복원하지 않습니다.',flush=True)
                raise RuntimeError('진단 라이다 종료 확인 필요')
        check=subprocess.run(['fuser',DEVICE],capture_output=True)
        if check.returncode!=1:
            print('라이다 포트가 사용 중이거나 확인에 실패해 중복 실행을 막았습니다. 기존 프로세스 상태를 확인하세요.',flush=True)
            raise RuntimeError('포트 해제 미확인: 자동 복원 중단')
        if log.exists():
            points=[]
            for line in log.read_text(errors='replace').splitlines():
                if 'SDK_NEAR ' in line:
                    try:points.append(json.loads(line.split('SDK_NEAR ',1)[1]))
                    except ValueError:pass
            print(json.dumps(dict(sdk_near_points=len(points),
                noise_flags=dict(Counter(str(d['noise_flag']) for d in points))),ensure_ascii=False),flush=True)
        print(f'진단 로그: {log}\n원래 라이다 실행으로 복원합니다. 이 터미널을 유지하세요.',flush=True)
        os.execv(str(ORIGINAL),args)


if __name__=='__main__':main()
