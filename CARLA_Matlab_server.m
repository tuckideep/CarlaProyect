%Direccion y puerto TCP/IP
t = tcpserver("192.168.1.141",3000);
t.ByteOrder = "big-endian";
configureCallback(t,"byte",1,@readFcn);

while true
pause(1);
end

function readFcn(src,~)
    if src.NumBytesAvailable>0
        RawData=read(src,src.NumBytesAvailable,"char");
    Type =split(RawData,"[[");
    ALLdata=split(Type(2),"]]");
    ref="]\n [";
    ref2=compose(ref);
    Array=split(ALLdata(1),ref2);
    length=size(Array);
    for x=1:1:length(1)
        %disp(Array(x));
        rawPoint =split(Array(x)," ");
        NumPoint =str2double(rawPoint); %1x 2y 3z 4v
        LengtNum=size(NumPoint);
        Point=0;
        index=1;
    
        for y=1:1:LengtNum(1)
            if ~isnan(NumPoint(y))
             Point(index)=NumPoint(y);
             index=index+1;
            end
        end

        Point3d.x=Point(1);
        Point3d.y=Point(2);
        Point3d.z=Point(3);
        Point3d.v=Point(4);
        disp(Point);
    end  

    %listPointCloud(x,:)=[Point(1),Point(2),Point(3)]; 
    %ListaDepuntos(x)=Point3d;  
    %doaction(src,ListaDepuntos)      
    %ptCloud =  pointCloud(listPointCloud);
    %player = pcplayer([-25 45], [-25 45],[-20 20]);

    %   while isOpen(player) 
    %     view(player,ptCloud);  
    %   end
    %doaction(src,ptCloud); 
    end   
end