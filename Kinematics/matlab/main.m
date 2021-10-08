clear interface % socket communication need to be cleared
interface = Interface();
while true
    cmd = interface.getVelCmd();% get local velocity command of robot coordination;
    disp("get velocity command : ");
    cmd;
    % cmd.velX cmd.velY,cmd.velW [-128,127]
    % TODO : do your ||kinematics|| here.
    
    %% constants
    theta1 = 53 / 180 * pi; % 53 degrees
    theta2 = 3 * pi / 4; % 135 degrees
    l = 154;
    r = 28;
    
    X_R_vel = [cmd.velX; cmd.velY; cmd.velW]; % velocity of robot in its own frame
    
    %% kinematics
    cmd.vel1 = [sin(theta1) -cos(theta1) -l] * X_R_vel / r;
    cmd.vel2 = [sin(-theta1) -cos(-theta1) -l] * X_R_vel / r;
    cmd.vel3 = [sin(-theta2) -cos(-theta2) -l] * X_R_vel / r;
    cmd.vel4 = [sin(theta2) -cos(theta2) -l] * X_R_vel / r;
    
    %% clip data into [-128,127]
    cmd.vel1(cmd.vel1 < -128) = -128;
    cmd.vel1(cmd.vel1 > 127) = 127;
    cmd.vel2(cmd.vel2 < -128) = -128;
    cmd.vel2(cmd.vel2 > 127) = 127;
    cmd.vel3(cmd.vel3 < -128) = -128;
    cmd.vel3(cmd.vel3 > 127) = 127;
    cmd.vel4(cmd.vel4 < -128) = -128;
    cmd.vel4(cmd.vel4 > 127) = 127;
    
    %% Control Wheels
    interface.controlWheel(cmd.vel1,cmd.vel2,cmd.vel3,cmd.vel4);
end