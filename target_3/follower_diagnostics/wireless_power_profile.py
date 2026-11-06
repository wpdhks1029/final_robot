#!/usr/bin/env python3
"""현재 Wi-Fi 프로필의 절전만 영구 해제. 연결 재시작·채널·출력·ROS 변경 없음."""
import argparse,json,re,socket,subprocess,time,uuid,shutil
from pathlib import Path


def active_uuid(text,iface):
    candidates=[]
    for line in text.splitlines():
        fields=line.split(':',1)
        if len(fields)==2 and fields[1]==iface:
            candidates.append(str(uuid.UUID(fields[0])))
    if len(candidates)!=1:raise ValueError('해당 인터페이스의 활성 프로필이 정확히 1개여야 합니다.')
    return candidates[0]


def power_value(text):
    value=text.strip().lower()
    aliases={'default':0,'ignore':1,'disable':2,'disabled':2,'enable':3,'enabled':3}
    if value in aliases:return aliases[value]
    m=re.match(r'^([0-3])(?=$|[\s(])',value)
    if not m:raise ValueError('NetworkManager 절전 값 형식을 지원하지 않습니다: '+repr(text))
    return int(m.group(1))


def runtime_power(text):
    m=re.search(r'Power save:\s*(on|off)',text)
    if not m:raise ValueError('무선 절전 상태를 읽을 수 없습니다.')
    return m.group(1)


def plan(iface,profile,value=2,runtime='off'):
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,32}',iface):raise ValueError('잘못된 인터페이스 이름')
    profile=str(uuid.UUID(profile))
    if type(value) is not int or value not in (0,1,2,3) or runtime not in ('on','off'):raise ValueError('잘못된 절전 값')
    return [['sudo','nmcli','connection','modify','uuid',profile,'802-11-wireless.powersave',str(value)],
            ['sudo','iw','dev',iface,'set','power_save',runtime]]


def read(args):
    return subprocess.run(args,check=True,stdout=subprocess.PIPE,text=True).stdout.strip()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',choices=('leader','follower'),required=True)
    p.add_argument('--iface',required=True)
    modes=p.add_mutually_exclusive_group();modes.add_argument('--apply',action='store_true');modes.add_argument('--restore',type=Path)
    a=p.parse_args()
    # 인터페이스를 명령에 전달하기 전에 형식을 검증한다.
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,32}',a.iface):p.error('잘못된 인터페이스 이름')
    if shutil.which('nmcli') is None:
        print('NetworkManager 명령 nmcli가 없어 프로필을 변경하지 않았습니다. 설치하지 마세요.')
        if shutil.which('iw') is not None:
            try:print(read(['iw','dev',a.iface,'get','power_save']))
            except subprocess.CalledProcessError:print('현재 무선 상태 조회도 실패했습니다. 설정은 변경하지 않았습니다.')
        print('이 장치는 별도 부팅 적용 서비스를 사용합니다. ROS·네트워크 재시작 없음.')
        return
    profile=active_uuid(read(['nmcli','-t','-f','UUID,DEVICE','connection','show','--active']),a.iface)
    old_profile=power_value(read(['nmcli','-g','802-11-wireless.powersave','connection','show','uuid',profile]))
    old_runtime=runtime_power(read(['iw','dev',a.iface,'get','power_save']))
    target_profile,target_runtime=2,'off'
    if a.restore:
        backup=json.loads(a.restore.read_text())
        if backup['host']!=socket.gethostname() or backup['iface']!=a.iface or backup['profile_uuid']!=profile:
            raise RuntimeError('백업의 호스트·인터페이스·현재 프로필이 일치하지 않습니다. 변경하지 않았습니다.')
        target_profile=backup['old_profile_powersave'];target_runtime=backup['old_runtime_powersave']
    commands=plan(a.iface,profile,target_profile,target_runtime)
    print(json.dumps(dict(host=socket.gethostname(),iface=a.iface,profile_uuid=profile,
        old_profile_powersave=old_profile,old_runtime_powersave=old_runtime,
        target_profile_powersave=target_profile,target_runtime_powersave=target_runtime,
        mode='RESTORE' if a.restore else ('APPLY' if a.apply else 'PREVIEW_ONLY')),ensure_ascii=False,indent=2),flush=True)
    if not a.apply and not a.restore:
        print('조회만 완료. --apply를 지정해야 변경합니다.');return
    directory=Path.home()/('leader_diagnostics' if a.role=='leader' else 'follower_diagnostics')/'logs'
    directory.mkdir(parents=True,exist_ok=True);path=directory/f'wireless_power_backup_{time.time_ns()}.json'
    path.write_text(json.dumps(dict(host=socket.gethostname(),iface=a.iface,profile_uuid=profile,
        old_profile_powersave=old_profile,old_runtime_powersave=old_runtime),ensure_ascii=False,indent=2)+'\n')
    print('BACKUP='+str(path),flush=True)
    if old_profile!=target_profile:subprocess.run(commands[0],check=True)
    if old_runtime!=target_runtime:subprocess.run(commands[1],check=True)
    current_profile=power_value(read(['nmcli','-g','802-11-wireless.powersave','connection','show','uuid',profile]))
    current_runtime=runtime_power(read(['iw','dev',a.iface,'get','power_save']))
    if (current_profile,current_runtime)!=(target_profile,target_runtime):raise RuntimeError('변경 후 상태가 목표와 다릅니다. 백업 파일과 출력을 확인하세요.')
    print(json.dumps(dict(profile_powersave=current_profile,runtime_power_save=current_runtime,reconnected=False),ensure_ascii=False),flush=True)
    print('절전 설정 확인 완료. 이 결과만으로 통신 지연 해결을 뜻하지 않습니다.',flush=True)

if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError,OSError,subprocess.CalledProcessError) as exc:
        print('절전 설정 작업 중단: '+str(exc),flush=True)
        raise SystemExit(2)
