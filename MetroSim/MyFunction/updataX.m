function [Tr,Plat,TimeSchedule]=updataX(time,k,TimeSchedule,Tr,Plat,control,noise)
%���³�����λ�ã������ж��Ƿ񵽴ﳵվ���Ƿ��ѵ����������

for i=1:Tr.M
    %����λ��
    
    Tr.TrX(i)=Tr.TrV(i)/k+Tr.TrX(i);
    if Tr.TrX(i)~=0
        Tr.first(i)=1;
    end
    if Tr.TrV(i)==0&&Tr.TrB(i)==true&&Tr.first(i)==1&&Tr.TrState(i)==0
        Tr.TrX(i)=getNearone(i,Tr,Plat);
    end
    
    %�ж��Ƿ񵽴��յ㣬��������¾�����վ
    if Tr.TrX(i)>=Plat.Distance&&Tr.TrPass(i)==Plat.N
        
        if Plat.PlatState(1)~=0
            Tr.TrV(i)=0;  %���ٶ���Ϊ0
            Tr.TrFuc(1:4,i)=0;  %���������ٶ���Ϊ0
        else
            [time,Tr]=putnoise(i,time,Tr,noise);
            Tr.TrX(i)=0;
            Tr.TrPass(i)=1;
            Plat.PlatState(1)=i;
            Tr.TrCir(i)=Tr.TrCir(i)+1;
            TimeSchedule.TimeTable(i,Tr.TrCir(i)*3+Tr.TrPass(i))=time;
            TimeSchedule.TimeArrive(i,Tr.TrCir(i)*6+2*Tr.TrPass(i)-1)=time;
            TimeSchedule.TimeError(i,Tr.TrCir(i)*3+Tr.TrPass(i))=time-TimeSchedule.TimeTable(i,Tr.TrCir(i)*3+1);
            TimeSchedule=updataTimeTable(i,Tr.TrPass(i),TimeSchedule,Tr,control);
            Tr.TrState(i)=1;
            Tr.TrIn(i)=time;
            Tr.TrV(i)=0;  %���ٶ���Ϊ0
            Tr.TrFuc(1:4,i)=0;  %���������ٶ���Ϊ0
            %��ճ˿ͣ��趨ͣվʱ��
            Tr.TrStay(i)=Plat.PlatNum(Tr.TrState(i))*Plat.P+Tr.Tmin;
            Plat.PlatNum(Tr.TrState(i))=0;
            TimeSchedule=updataTimePlan(i,Tr,TimeSchedule,Plat);

            continue;
        end
    end
    if Tr.TrPass(i)~=Plat.N     
        if Tr.TrX(i)>=Plat.PlatX(Tr.TrPass(i)+1)
            if Plat.PlatState(Tr.TrPass(i)+1)~=0
                Tr.TrV(i)=0;  %���ٶ���Ϊ0d
                Tr.TrFuc(1:4,i)=0;  %���������ٶ���Ϊ0
            else
                [time,Tr]=putnoise(i,time,Tr,noise);
                Tr.TrX(i)=Plat.PlatX(Tr.TrPass(i)+1);
                Tr.TrPass(i)=Tr.TrPass(i)+1;
                Plat.PlatState(Tr.TrPass(i))=i;
                TimeSchedule.TimeError(i,Tr.TrCir(i)*3+Tr.TrPass(i))=time-TimeSchedule.TimeTable(i,Tr.TrCir(i)*3+Tr.TrPass(i));
                TimeSchedule.TimeTable(i,Tr.TrCir(i)*3+Tr.TrPass(i))=time;
                TimeSchedule.TimeArrive(i,Tr.TrCir(i)*6+2*Tr.TrPass(i)-1)=time;
                TimeSchedule=updataTimeTable(i,Tr.TrPass(i),TimeSchedule,Tr,control);
                
                Tr.TrState(i)=Tr.TrPass(i);
                Tr.TrIn(i)=time;
                Tr.TrV(i)=0;   %���ٶ���Ϊ0
                Tr.TrFuc(1:4,i)=0;   %���������ٶ���Ϊ0 
                %�趨ͣվʱ��(����Ԥ����վ̨�ڼ����ĳ˿�)
                Tr.TrStay(i)=(Plat.PlatNum(Tr.TrState(i))*Plat.P+Tr.Tmin)/(1-Plat.ck(Tr.TrPass(i))*Plat.P);
                TimeSchedule=updataTimePlan(i,Tr,TimeSchedule,Plat);

            end
        end
    end
end
