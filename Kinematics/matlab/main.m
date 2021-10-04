clear interface % socket communication need to be cleared
interface = Interface();
while true
    cmd = interface.getVelCmd();% get local velocity command of robot coordination;
    disp("get velocity command : ");
    % cmd
    % cmd.velX cmd.velY,cmd.velW [-128,127]
    % TODO : do your ||kinematics|| here.
    % interface.controlWheel(cmd.velX,cmd.velX,cmd.velY,cmd.velY);
    interface.controlWheel(48,0,0,0);
end