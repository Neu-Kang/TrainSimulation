function [Tr,TimeSchedule]=GetPlan2(TimeSchedule,time,Tnum,Pnum,TimeTable,Plat,Tr)
%�������ܣ����ڵ����µ�վ̨���������µ��������
syms tx;

aTr=Tr.TraTr;
if TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))<TimeSchedule.MinRunTime(Pnum)
    duik=TimeSchedule.MinRunTime(Pnum)-TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum));
    TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.MinRunTime(Pnum);
    TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+duik;
    TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))+duik;
end
T=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum));
% T=TimeTable(Tnum,Pnum+1+Plat.N*Tr.TrCir(Tnum))-time;
if T<sqrt(4*Plat.PlatDis(Pnum)/aTr)
   cuik=sqrt(4*Plat.PlatDis(Pnum)/aTr)-T;
   T=sqrt(4*Plat.PlatDis(Pnum)/aTr);
   begintime=time+T;
   TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=begintime;
   TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=ceil(arrivebybegin(Tr,Plat,Tnum,begintime,TimeSchedule.BeginTimeTable));
   TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
   TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+cuik;
   fprintf("�г�%d��վ%dʱ��Ϊ%f",Tnum,Pnum,time);
end
t=double(solve((aTr*tx*(T-tx)-Plat.PlatDis(Pnum)),tx));
t1=t(1,1);
t2=t(2,1);
v=aTr*t1;
b2=aTr*T;
Tr.TrFuc(1,Tnum)=aTr;
Tr.TrFuc(3,Tnum)=-aTr;
Tr.TrFuc(2,Tnum)=v;
Tr.TrFuc(4,Tnum)=b2;
Tr.TrCh(1,Tnum)=t1+time;
Tr.TrCh(2,Tnum)=t2+time;
Tr.TrBT(1,Tnum)=time;
Tr.TrAT(1,Tnum)=time+T;
% if Tnum==1
%     Tr.TrCh(1,Tnum)
%     Tr.TrCh(2,Tnum)
%     v;
%     b2;
%     y=1;
% end
    
