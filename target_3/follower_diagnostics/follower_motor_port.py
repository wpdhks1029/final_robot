"""Exclusive serial adapter. Importing this module does not open a device."""
import fcntl
import os
import stat
import subprocess
import termios

from follower_motor_candidate import read_request,velocity_packet


class SingleWriteSerial:
    """단일 비차단 OS 쓰기. 부분 전송·EAGAIN·EINTR은 상위 계층에서 오류 처리."""
    timeout=0
    write_timeout=0

    def __init__(self,device):
        self.device=device
        self.fd=device.fileno()
        if not fcntl.fcntl(self.fd,fcntl.F_GETFL)&os.O_NONBLOCK:
            raise RuntimeError('비차단 시리얼 fd만 허용합니다.')

    def fileno(self):return self.fd
    @property
    def in_waiting(self):return self.device.in_waiting
    def read(self,n):return self.device.read(n)
    def reset_input_buffer(self):return self.device.reset_input_buffer()
    def close(self):return self.device.close()
    def write(self,packet):
        # pySerial의 쓰기 후 select/시간 초과 판정을 사용하지 않는다.
        # OS 큐 수락만 뜻하며 실제 송신·응답·정지를 보장하지 않는다.
        return os.write(self.fd,packet)


def owners(port):
    # psmisc fuser on the robot rejects '--'; only absolute device paths are accepted.
    if not isinstance(port,str) or not port.startswith('/dev/'):
        raise ValueError('Ownership check requires an absolute /dev/ path')
    result=subprocess.run(['fuser',port],capture_output=True,text=True,timeout=2)
    if result.returncode==1 and not result.stdout.strip() and not result.stderr.strip():
        return set()
    if result.returncode!=0:raise RuntimeError('Cannot verify serial ownership: '+result.stderr.strip())
    tokens=result.stdout.split()
    if not tokens or not all(t.isdigit() for t in tokens):
        raise RuntimeError('Unrecognized fuser output')
    return {int(t) for t in tokens}


class ExclusivePort:
    """Pre/post owner checks plus flock and kernel TIOCEXCL. Not a hardware watchdog."""
    timeout=0
    write_timeout=0

    def __init__(self,serial_device):
        self.device=serial_device
        self.closed=False

    @classmethod
    def open(cls,port,baudrate=19200):
        # All calls here are physical I/O and require explicit operator authorization.
        if baudrate!=19200:raise ValueError('Follower adapter expects confirmed 19200bps')
        canonical=os.path.realpath(port)
        if not stat.S_ISCHR(os.stat(canonical).st_mode):raise RuntimeError('Port is not a character device')
        busy=owners(canonical)
        if busy:raise RuntimeError(f'Port already owned by PID(s) {sorted(busy)}; refusing open')
        import serial
        device=serial.Serial(port=canonical,baudrate=baudrate,bytesize=8,parity='N',stopbits=1,
                             timeout=0,write_timeout=0,exclusive=True)
        exclusive=False
        try:
            fcntl.ioctl(device.fileno(),termios.TIOCEXCL)
            exclusive=True
            # Check processes which may have opened before TIOCEXCL was applied.
            after=owners(canonical)
            if after!={os.getpid()}:
                raise RuntimeError(f'Port ownership changed or cannot be verified: {sorted(after)}')
            return cls(SingleWriteSerial(device))
        except Exception:
            if exclusive:
                try:fcntl.ioctl(device.fileno(),termios.TIOCNXCL)
                except OSError:pass
            device.close()
            raise

    @property
    def in_waiting(self):return self.device.in_waiting
    def read(self,n):return self.device.read(n)
    def write(self,packet):
        # This first physical adapter is independently locked at the byte boundary.
        # Even a mistaken upstream state cannot send nonzero velocity or other PIDs.
        if packet not in (read_request(1),velocity_packet(0,0,1)):
            raise ValueError('Locked port permits ID1 PID4 or ENABLE=1 zero-RPM packet only')
        return self.device.write(packet)
    def reset_input_buffer(self):return self.device.reset_input_buffer()

    def close(self):
        if self.closed:return
        try:fcntl.ioctl(self.device.fileno(),termios.TIOCNXCL)
        finally:
            self.device.close();self.closed=True
