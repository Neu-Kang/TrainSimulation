function Tr=getspeed(t,Tr,k)
%�������ܣ��õ���ǰʱ���г������ٶ�
t=t+1/k;%Ԥ�������һ�����ٶȣ�ͬʱ�����ٶ�Ϊ0�����λ�������ͻ
for i=1:Tr.M
    if Tr.TrB(i)==false
         Tr.TrV(i)=0;
         continue
    end
    if t<Tr.TrCh(1,i)
        Tr.TrV(i)=Tr.TrFuc(1,i)*(t-Tr.TrBT(1,i));
    elseif t>=Tr.TrCh(1,i)&&t<=Tr.TrCh(2,i)
        Tr.TrV(i)=Tr.TrFuc(2,i);
    else
        Tr.TrV(i)=Tr.TrFuc(3,i)*(t-Tr.TrBT(1,i))+Tr.TrFuc(4,i);
    end
    if  Tr.TrV(i)<0
        Tr.TrV(i)=0;
    end
    %�����ٶȲ��ܴ���Ѳ���ٶȲ��ܵ���0
    if Tr.TrV(i)>Tr.TrFuc(2,i)
        Tr.TrV(i)=Tr.TrFuc(2,i);
    end
    if Tr.TrV(i)<0
        Tr.TrV(i)=0;
    end
end

    


