function [Tr,Plat,TimeSchedule]=checkout(time,TimeSchedule,Plat,Tr)
%��������:����Ƿ��г�վ�г�(������С�г����)�����г�վ������վ̨���г�״̬��վ̨��������,�����µ��г������ٶ�
for i=1:Tr.M
    if Tr.TrState(i)~=0 
        if (time-Tr.TrIn(i))>=Tr.TrStay(i)
            if checksafeinterval(time,i,Tr,Plat)
                TimeSchedule.TimeArrive(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)))=time;
                Tr=GetPlan2(time,i,Tr.TrState(i),TimeSchedule.TimeTable,Plat,Tr); %�õ��µ��г������ٶ�
                Plat.timelast(Tr.TrState(i))=time;
                Plat.PlatState(Tr.TrState(i))=0;
                Plat.PlatNum(Tr.TrState(i))=0;
                Tr.TrState(i)=0;
                Tr.TrIn(i)=0;
                Tr.TrStay(i)=0; 
            end
        end
    end
end