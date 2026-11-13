#!/usr/bin/env python3
"""SSH와 독립된 tmux 창에서 기존 기준 명령을 실행. 기본은 명령 미리보기."""
import argparse,json,os,shlex,shutil,subprocess
from pathlib import Path


def commands(role,stage,seconds,home):
    h=Path(home);d=h/('leader_diagnostics' if role=='leader' else 'follower_diagnostics')
    if seconds not in (30,60):raise ValueError('상한은 30/60초만 지원합니다.')
    if role=='leader':
        if stage!='core':raise ValueError('선두는 core만 사용합니다.')
        return [
          ('base',['ros2','launch',str(d/'leader_nav_geometry_candidate.launch.py'),'allow_motion:=true']),
          ('localization',['ros2','launch','nav2_bringup','localization_launch.py',f'map:={h}/robot_ws/maps/leader_map.yaml',f'params_file:={d}/nav2_015_rotation_shim_check.yaml','use_sim_time:=false','autostart:=true']),
          ('bridge',['python3','-B',str(d/'leader_bridge_latest_candidate.py'),'--ros-args','-p','map_frame:=map','-p','base_frame:=base_link','-p','publish_rate:=20.0'])]
    if stage=='core':return [
          ('motor',['python3','-B',str(d/'follower_drive_node.py'),'--run-hardware','--ack-hardware-zero-rpm-tx','--input-kind','guard','--enable-motion-input','--trial-max-rpm','10','--trial-seconds',str(seconds)]),
          ('sensors',['ros2','launch',str(d/'follower_position_fixed_lidar.launch.py'),'enable_position_profile:=true']),
          ('localization',['ros2','launch',str(d/'follower_localization.launch.py'),'enable_localization:=true']),
          ('bridge',['python3','-B',str(h/'robot_ws/src/cnu_capstone_2026/convoy_robot/real_follower_bridge_tf_candidate.py')])]
    return [('preview',['ros2','launch',str(d/'follower_convoy_preview.launch.py'),'enable_preview:=true']),
            ('gateway',['python3','-B',str(d/'follower_guard_gateway.py'),'--run'])]


def shell_command(argv,home,delay):
    # shell에 전달하는 경로와 모든 인수는 shlex로 인용한다.
    body='\n'.join(['source /opt/ros/humble/setup.bash',
         'source '+shlex.quote(str(Path(home)/'robot_ws/install/setup.bash')),
         'export ROS_DOMAIN_ID=10','export ROS_LOCALHOST_ONLY=0',
         'sleep '+str(delay),'exec '+shlex.join(argv)])
    return shlex.join(['/bin/bash','-c',body])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--role',choices=('leader','follower'),required=True)
    p.add_argument('--stage',choices=('core','control'),default='core');p.add_argument('--trial-seconds',type=int,choices=(30,60),default=30)
    p.add_argument('--start',action='store_true');a=p.parse_args();home=Path.home()
    steps=commands(a.role,a.stage,a.trial_seconds,home);session='convoy_'+a.role
    if not a.start:
        print(json.dumps(dict(session=session,commands=[dict(window=n,argv=c) for n,c in steps],mode='PREVIEW_ONLY'),ensure_ascii=False,indent=2));return
    if not shutil.which('tmux'):raise RuntimeError('tmux가 없어 실행하지 않았습니다. ROS 프로세스는 변경하지 않았습니다.')
    # 새 세션 전에 이미 실행 중인 기준 프로세스가 있으면 중복 기동을 막는다.
    exists=subprocess.run(['tmux','has-session','-t',session],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
    if not exists:
        markers=['leader_nav_geometry_candidate.launch.py','leader_nav_015_base.launch.py','real_leader_bridge_node.py','leader_bridge_latest_candidate.py'] if a.role=='leader' else ['follower_drive_node.py','follower_motor_locked.py','follower_position_observation.launch.py','follower_position_fixed_lidar.launch.py','start_lidar_fixed_candidate.py','lidar_sdk_trace_node','follower_localization.launch.py','real_follower_bridge_tf_candidate.py','follower_convoy_preview.launch.py','follower_guard_gateway.py']
        found=[]
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:args=path.read_bytes().split(b'\0')
            except OSError:continue
            if int(path.parent.name)==os.getpid():continue
            names=[Path(os.fsdecode(x)).name for x in args if x]
            if any(m in names for m in markers):found.append(path.parent.name)
        if found:raise RuntimeError('기존 ROS 프로세스가 있어 새 세션을 시작하지 않았습니다. 기존 실행을 먼저 종료하세요. PID='+','.join(found))
    if a.role=='follower' and a.stage=='control' and not exists:raise RuntimeError('먼저 core 단계와 초기 pose를 준비하세요.')
    # 원격 호스트 파일 존재를 실제 실행 전에 확인한다.
    for name,cmd in steps:
        for arg in cmd:
            if arg.startswith(str(home)+'/') and not Path(arg).exists():raise RuntimeError('필수 파일 없음: '+arg)
    for i,(name,cmd) in enumerate(steps):
        if exists:
            rows=subprocess.check_output(['tmux','list-panes','-s','-t',session,'-F','#{window_name}:#{pane_dead}'],text=True).splitlines()
            matched=[r for r in rows if r.split(':',1)[0]==name]
            if matched:
                print(name+': 이미 존재. 재시작하지 않습니다. '+('종료된 창이므로 로그를 확인하세요.' if matched[0].endswith(':1') else ''),flush=True);continue
        delay=i*4 if a.stage=='core' else i*2
        command=shell_command(cmd,home,delay)
        if not exists:
            subprocess.run(['tmux','new-session','-d','-s',session,'-n',name,command],check=True);exists=True
        else:subprocess.run(['tmux','new-window','-d','-t',session,'-n',name,command],check=True)
        subprocess.run(['tmux','set-window-option','-t',session+':'+name,'remain-on-exit','on'],check=True)
        print('시작 창: '+session+':'+name,flush=True)
    print('접속: tmux attach -t '+session,flush=True)
    print('분리: Ctrl+B를 누른 뒤 D. SSH 종료 후에도 프로세스는 유지됩니다. 자동 arm/주행/재시작은 없습니다.',flush=True)

if __name__=='__main__':
    try:main()
    except (RuntimeError,ValueError,OSError,subprocess.CalledProcessError) as exc:
        print('시작 중단: '+str(exc),flush=True);raise SystemExit(2)
