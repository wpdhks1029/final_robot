#!/usr/bin/env python3
"""Read MD200T fault registers via PID4 only; no drive/reset/settings writes.

Manufacturer RS485 specification V6.4h pp33/35/45/94/99. Alarm log reply
length varies by firmware: preserve bytes and leave unsupported replies unknown.
Stop the existing serial owner before running on a stationary robot.
"""
import json
import sys
import read_md200t_settings as reader

FAULT_FIELDS = {
    34: ('motor1_control_status', 1),
    39: ('motor2_control_status', 1),
    60: ('latest_alarm_status_firmware_dependent', 1),
    99: ('all_current_status_dual_motor', 8),
    229: ('alarm_log_firmware_dependent', 12),
}


def describe(pid,frame,result):
    result=dict(result)
    if frame is None:return result
    data=frame[5:-1]
    result['data_bytes']=list(data)
    if result.get('result')!='OK':return result
    if pid==99 and len(data)==8:
        names=('ALARM','CTRL_FAIL','OVER_VOLT','OVER_TEMP','OVER_LOAD','HALL_FAIL','INV_VEL','STALL')
        names2=('REGEN_OVER_TEMP','ENC_FAIL','SYNC_CTRL_FAIL','SYNC_COM_FAIL','FAULT','UNDER_VOLT','SHUT_OVER_LOAD','MENA_FAIL')
        result['current_motor_status']={
            motor:dict(status=data[offset],status2=data[offset+1],
                       flags=[name for bit,name in enumerate(names) if data[offset]&(1<<bit)],
                       flags2=[name for bit,name in enumerate(names2) if data[offset+1]&(1<<bit)])
            for motor,offset in (('m1',0),('m2',4))}
    elif pid==229 and len(data)==12:
        result['historical_counts']=dict(zip(
            ('CTRL_FAIL','OVER_VOLT','OVER_TEMP','SHORT','OVER_LOAD','HALL_FAIL','INV_VEL','STALL','ENC_FAIL','UNDER_VOLT'),data[1:11]))
        result['latest_alarm_raw']=data[0]
    return result


def main():
    # Reuse the established exclusive port, PID4 whitelist, frame checksum,
    # finite query timeout, and zero-RPM checks. Change no source settings.
    original = reader.FIELDS
    original_value = reader.value
    reader.FIELDS = dict(FAULT_FIELDS)
    reader.value=lambda pid,frame:describe(pid,frame,original_value(pid,frame))
    try:
        return reader.main()
    finally:
        reader.FIELDS = original
        reader.value = original_value


if __name__ == '__main__':
    sys.exit(main())
