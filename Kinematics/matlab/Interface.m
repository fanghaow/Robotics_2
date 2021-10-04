classdef Interface
    properties
        recv_port = 45454
        send_port = 45455
        socket
    end
    methods
        function obj = Interface()
            % there's only one interface in the initailization stage.
            obj.socket = udpport("datagram","IPV4","LocalPort", obj.recv_port);
        end
        function data = getVelCmd(obj)
            while true
                if (obj.socket.NumDatagramsAvailable > 0)
                    datagram = read(obj.socket, obj.socket.NumDatagramsAvailable, "int8");
                    data = struct();
                    datagram;
                    data.velX = datagram(1).Data(1);
                    data.velY = -datagram(1).Data(2);
                    data.velW = datagram(1).Data(3);
                    return
                end
            end
        end
        function controlWheel(obj,v1,v2,v3,v4)
            % TODO convert from floating to uint8
            d = [v1,v2,v3,v4]*60/(2*math.pi)/2.5;
            d(d>127) = 127;
            d(d<-127) = -127;
            data = int8(d);
            data
            write(obj.socket,data,"int8","127.0.0.1",obj.send_port);
        end
        function delete(obj)
            clear obj.socket;
        end
    end
end
