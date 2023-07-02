import camera

for i in camera.idlist():
    print("id: {n}, serial: {s}".format(n=i, s=camera.id2serial(i)))
