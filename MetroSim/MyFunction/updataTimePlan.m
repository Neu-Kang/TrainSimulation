function TimeSchedule=updataTimePlan(i,Tr,TimeSchedule,Plat)
%�������ܣ����¼ƻ�����ʱ�̱�����ǰһ������������ѭ��С��ȫʱ�����������
num=cirsub(i,Tr.M);
%����ڱ�����ǰ���Ƿ��б�ĳ�
%% ���ǰ���������г��������������� ����Ҫ���ǰ�����ķ����뱾���ķ�������밲ȫ���
if Tr.TrPass(i)==Tr.TrPass(num)&&Tr.TrB(num)==true&&Tr.TrX(i)<Tr.TrX(num)%�����Ϊǰ���г�����������
    %���ǰ���г����ƻ�����ʱ��=�ƻ�����ʱ��+(�ƻ�����ʱ��-��һ����Ԥ����һվ̨�ƻ�����ʱ��)*���������ٶ�*������ʱ���ϵ+��Сͣ��ʱ�䣩/��1-���������ٶ�*������ʱ���ϵ��
    %������ȡ��
    TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)-1)+...
        ((TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)-1)-TimeSchedule.TimePlan(num,Tr.TrCir(num)*6+2*(Tr.TrPass(num)+1)))*Plat.P*Plat.ck(ciradd(Tr.TrPass(i),Plat.N))+Tr.Tmin)/...
        (1-Plat.ck(ciradd(Tr.TrPass(i),Plat.N))*Plat.P);
    TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=ceil(TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)));
    if TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))-TimeSchedule.TimePlan(num,Tr.TrCir(num)*6+2*(Tr.TrPass(num)+1))<Plat.Tinterval
        TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=TimeSchedule.TimePlan(num,Tr.TrCir(num)*6+2*(Tr.TrPass(num)+1))+Plat.Tinterval;
    end
    return
end
%% ���ǰ����վ�г�������վ̨ͣ�� ����Ҫ���ǰ�����ķ����뱾���ķ�������밲ȫ���
if ciradd(Tr.TrPass(i),Plat.N)==Tr.TrPass(num)&&Tr.TrB(num)==true&&Tr.TrX(i)<Tr.TrX(num)&&Tr.Trstate==1%�����Ϊǰ���г���վ̨ͣ��
    %���վ̨�г����ƻ�����ʱ��=�ƻ�����ʱ��+(�ƻ�����ʱ��-��һ������ǰվ̨�ƻ�����ʱ��)*���������ٶ�*������ʱ���ϵ+��Сͣ��ʱ�䣩/��1-���������ٶ�*������ʱ���ϵ��
    %������ȡ��
    TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)-1)+...
        ((TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)-1)-TimeSchedule.TimePlan(num,Tr.TrCir(num)*6+2*Tr.TrPass(num)))*Plat.P*Plat.ck(ciradd(Tr.TrPass(i),Plat.N))+Tr.Tmin)/...
        (1-Plat.ck(ciradd(Tr.TrPass(i),Plat.N))*Plat.P);
    TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=ceil(TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)));
    if TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))-TimeSchedule.TimePlan(num,Tr.TrCir(num)*6+2*Tr.TrPass(num))<Plat.Tinterval
        TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=TimeSchedule.TimePlan(num,Tr.TrCir(num)*6+2*Tr.TrPass(num))+Plat.Tinterval;
    end
    return
end
%% ���ǰ�������޳� ����Ҫ���վ̨�ѹ������뱾���ķ�������밲ȫ������Ա�
TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)-1)+...
    (((TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)-1)-Tr.TrIn(i))*Plat.ck(ciradd(Tr.TrPass(i),Plat.N))+Plat.PlatNum(ciradd(Tr.TrPass(i),Plat.N)))*Plat.P+Tr.Tmin)/...
    (1-Plat.ck(ciradd(Tr.TrPass(i),Plat.N))*Plat.P);
%����ȡ��������ʱ��ֻ��ȡ��
TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=ceil(TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1)));
if TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))-Plat.timelast(ciradd(Tr.TrPass(i),Plat.N))<Plat.Tinterval 
    TimeSchedule.TimePlan(i,Tr.TrCir(i)*6+2*(Tr.TrPass(i)+1))=Plat.timelast(ciradd(Tr.TrPass(i),Plat.N))+Plat.Tinterval;
end

