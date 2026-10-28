import threading
from types import SimpleNamespace
from rm_flatness_nav.driver import Driver

def test_expired_tof_does_not_become_a_current_range():
    fake=SimpleNamespace(lock=threading.Lock(),latest={'esc':(10,[0]*4),'tof':(9,[150,0,0,0])},
                         video=SimpleNamespace(stamp=10),last_command_ack=True,
                         last_command_ack_stamp=10,cfg={'front_tof_index':0})
    healthy,tof,_=Driver.health(fake,10)
    assert healthy and tof is None
    fake.latest['tof']=(9.9,[150,0,0,0])
    assert Driver.health(fake,10)[1]==.15
