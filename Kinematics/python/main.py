from interface import Interface
interface = Interface()
print("start interface...")
while True:
    cmd = interface.getVelCmd()
    print("get velocity command : ",cmd.velX,cmd.velY,cmd.velW)
    # TODO : do your kinematics here
    interface.controlWheel(0,0,0,6.28)
