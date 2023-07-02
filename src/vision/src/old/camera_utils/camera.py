import os
import subprocess

def idlist():
    p = subprocess.Popen(
            f"ls /dev/ | grep video",
            stdout=subprocess.PIPE,
            shell=True,
        )
    out, _ = p.communicate()
    p.wait()
    s=out.decode("utf-8").split()
    ret=[]
    for i in s:
        ret.append(i[5:])

    return ret

def id2serial(id):
    p = subprocess.Popen(
        f"udevadm info --name=/dev/video{id} | grep ID_SERIAL_SHORT=",
        stdout=subprocess.PIPE,
        shell=True,
    )
    out, _ = p.communicate()
    p.wait()
    return out.decode("utf-8").strip()[-8:]


def serial2id(serial):
    id = None
    for i in range(16):
        if serial == id2serial(i):
            id = i
            break
    return id
