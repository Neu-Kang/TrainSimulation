function [Tr,Plat,TimeSchedule,ylast]=checkout2(time,TimeSchedule,Plat,Tr,control,ylast)
%��������:����Ƿ��г�վ�г�(������С�г����)�����г�վ������վ̨���г�״̬��վ̨��������,�����µ��г������ٶ�
for i=1:Tr.M
    if Tr.TrState(i)~=0 
        if (time-Tr.TrIn(i))>=Tr.TrStay(i)
            if checksafeinterval(time,i,Tr,Plat)
                if Tr.TrPass(i)==2&&i==1              
                    Tr.firstcircle=true;
                end
                Plat.timelast(Tr.TrState(i))=time;
                TimeSchedule.TimeArrive(i,Tr.TrCir(i)*Plat.N*2+2*(Tr.TrPass(i)))=time;
                TimeSchedule.TimeError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.BeginTimeTable(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i));
                if i==1 %��һȦ�г�һ������Ϊ0���ڶ�Ȧ��ʼ����
                    if Tr.TrCir(i)==0
                        TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=0;
                    else
                        TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.BeginTimeTable(Tr.M,(Tr.TrCir(i)-1)*Plat.N+Tr.TrPass(i))-Plat.DepartInter;
                    end
                else 
                    TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.BeginTimeTable(i-1,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))-Plat.DepartInter;
                end
                if control.sort==0 %��ʱ�̱����
                    TimeSchedule=updataTimeTable2(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==1 %��ʱ�̱����
                    TimeSchedule=updataTimeTable3(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                [Tr,TimeSchedule]=GetPlan2(TimeSchedule,time,i,Tr.TrState(i),TimeSchedule.TimeTable,Plat,Tr); %�õ��µ��г������ٶ�
                Plat.PlatState(Tr.TrState(i))=0;
                Plat.PlatNum(Tr.TrState(i))=0;
                Tr.TrState(i)=0;
                Tr.TrIn(i)=0;
                Tr.TrStay(i)=0; 
%                 [ylast,ylast2,judge]=checkOver(Tr,ylast);6
%                 if judge==1
%                    disp('����Խ��')
%                    ylast2
%                    ylast
%                 end
            end
        end
    end
end