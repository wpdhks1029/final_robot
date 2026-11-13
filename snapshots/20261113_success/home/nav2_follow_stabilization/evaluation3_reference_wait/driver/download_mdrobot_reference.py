"""Download manufacturer's linked reference PDF only; no ROS/serial access."""
from pathlib import Path
import urllib.request


def main():
    url='https://drive.google.com/uc?export=download&id=1n9jZexVmjpyT_lGQIhbAzJql6ix77np5'
    target=Path(__file__).with_name('mdrobot_rs485_reference.pdf')
    try:
        with urllib.request.urlopen(url,timeout=30) as response:
            data=response.read(20_000_001)
        if len(data)>20_000_000:raise ValueError('File exceeds 20MB limit')
        if not data.startswith(b'%PDF-'):
            raise ValueError('Response is not PDF; existing file left unchanged')
        target.write_bytes(data)
        print(f'PDF saved: {target} ({len(data)} bytes)')
    except Exception as exc:
        print(f'Download failed: {exc}')
        return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
