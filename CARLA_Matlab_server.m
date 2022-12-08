%%Direccion y puerto TCP/IP
t = tcpserver("192.168.1.141",3000);
t.ByteOrder = "big-endian";
%%LLamado al Callback de la función readFcn
configureCallback(t,"byte",1,@readFcn);
while true
    pause(1) 
end  
%%Función de decifrado de datos
function readFcn(src,~)
    if src.NumBytesAvailable>0
        RawData=read(src,src.NumBytesAvailable,"char");
        %%Condicion si hay caracteres demás
        lengthRaw=size(RawData);
        if lengthRaw(2)>50
            %%Orden de datos
            Type =split(RawData,"[[");
            ALLdata=split(Type(2),"]]");
            ref="]\n [";
            ref2=compose(ref);
            Array=split(ALLdata(1),ref2);
            length=size(Array);
        %
        for x=1:1:length(1)
            rawPoint =split(Array(x)," ");
            NumPoint =str2double(rawPoint); %1x 2y 3z 4v
            LengtNum=size(NumPoint);
            Point=0;
            index=1;
            %
            for y=1:1:LengtNum(1)
                if ~isnan(NumPoint(y))
                 Point(index)=NumPoint(y);
                 index=index+1;
                end
            end
            %%Puntos 3D separados por ejes
            Point3d.x=Point(1);
            Point3d.y=Point(2);
            Point3d.z=Point(3);
            Point3d.v=Point(4);
            %disp(Point); 
            velocidad = Point(4);
            %disp(Point(1));
            listPointCloud(x,:)=[Point(1),Point(2),Point(3)];
        end 
        %disp(listPointCloud);
        %%Llamada de función
        doaction(src,listPointCloud,velocidad)
        end
    end  
end

