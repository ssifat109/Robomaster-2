"""Read-only stream transport diagnostic. Never commands chassis or arm."""
import socket,time,json
from pathlib import Path
from robomaster import robot,config

class Capture:
    def start_video_stream(self,display,addr,protocol):
        self.addr=addr; self.protocol=protocol
        return True
    def stop_video_stream(self): return True
    def stop(self): return True

out=Path(__file__).resolve().parents[1]/'evidence/video_transport'
out.mkdir(parents=True,exist_ok=True)
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
    s.connect(('192.168.2.1',20020)); config.LOCAL_IP_STR=s.getsockname()[0]
ep=robot.Robot(); receiver=Capture()
try:
    assert ep.initialize(conn_type='ap')
    ep.camera._liveview=receiver
    assert ep.camera.start_video_stream(display=False,resolution='720p')
    print('stream',receiver.addr,receiver.protocol,flush=True)
    time.sleep(1)
    data=bytearray()
    with socket.create_connection(receiver.addr,timeout=3) as sock:
        sock.settimeout(3)
        start=time.monotonic()
        while time.monotonic()-start<4:
            try: block=sock.recv(65536)
            except socket.timeout: break
            if not block: break
            data.extend(block)
    (out/'stream.h264').write_bytes(data)
    result={'bytes':len(data),'prefix_hex':data[:32].hex(),'local_ip':config.LOCAL_IP_STR}
    (out/'result.json').write_text(json.dumps(result,indent=2)); print(result,flush=True)
finally:
    try: ep.camera.stop_video_stream()
    finally: ep.close()
