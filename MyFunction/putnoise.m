function [ptime,Tr]=putnoise(i,j,k,time,Tr,Plat,noise)
%�������ܣ�������ʱ����Ӹ���
ptime=time;
switch noise
    case -1
        ptime=time+roundn(1*rand,-2);
    case 0
        ptime=time;
    case 1
        if i==2&&j==2
           ptime=time+2;
           fprintf("�г�%d��վ̨%d������Ϊ%f",i,j,15);
        end
        if i==3&&j==6
            ptime=time+0;
            fprintf("�г�%d��վ̨%d������Ϊ%f",i,j,0);
        end
    case 2
        if i==2&&j==2
            ptime=time+7;
        end
    case 3
         ptime=time;
end