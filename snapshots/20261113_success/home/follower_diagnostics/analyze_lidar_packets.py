#!/usr/bin/env python3
"""진단 SDK가 저장한 G2B 8비트 원본 패킷을 독립적으로 검증한다."""
import argparse,json
from pathlib import Path


def decode(hex_text):
    data=bytes.fromhex(hex_text)
    if len(data)<10 or data[:2]!=b'\xaa\x55':raise ValueError('패킷 헤더 불일치')
    count=data[3]
    if count<1 or len(data)!=10+count*3:raise ValueError('8비트 강도 패킷 길이 불일치')
    checksum=0
    for i in range(0,8,2):checksum^=int.from_bytes(data[i:i+2],'little')
    points=[]
    for i in range(count):
        pos=10+i*3;quality=data[pos];raw=int.from_bytes(data[pos+1:pos+3],'little')
        checksum^=quality;checksum^=raw
        points.append(dict(sample_index=i,quality=quality,raw_distance=raw&0xfffc,range_m=(raw&0xfffc)/4000.,noise_flag=raw&3))
    original=int.from_bytes(data[8:10],'little')
    return dict(checksum_ok=checksum==original,calculated_checksum=checksum,packet_checksum=original,points=points)


def analyze(path):
    rows=[];errors=[]
    for line in path.read_text(errors='replace').splitlines():
        if 'SDK_PACKET_NEAR ' not in line:continue
        try:
            row=json.loads(line.split('SDK_PACKET_NEAR ',1)[1]);d=decode(row['packet_hex']);point=d['points'][row['sample_index']]
            matches=all(point[k]==row[k] for k in ('quality','raw_distance','noise_flag'))
            rows.append(dict(packet_stamp_ns=row['packet_stamp_ns'],sample_index=row['sample_index'],checksum_ok=d['checksum_ok'],decoded_matches_sdk=matches,**{k:point[k] for k in ('quality','raw_distance','range_m','noise_flag')}))
        except (ValueError,KeyError,IndexError,TypeError) as exc:errors.append(str(exc))
    return dict(points=rows,errors=errors,total_near_points=len(rows),bad_checksum=sum(not r['checksum_ok'] for r in rows),decode_mismatch=sum(not r['decoded_matches_sdk'] for r in rows))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('log',type=Path);a=p.parse_args();result=analyze(a.log)
    out=a.log.with_suffix('.packets.json');out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='points'},ensure_ascii=False,indent=2));print('JSON='+str(out))


if __name__=='__main__':main()
