function TimeSchedule=updataTimeTable5(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%更新到达车辆的时刻表，函数追求的目标是与目标（带名义时刻表场景）
%计算控制率
error=TimeSchedule.TimeError(Tnum,Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum));
t1=clock;
if Tnum==1&& Pnum==3&&Tr.TrCir(Tnum)==1
    z=11;
end
%加入控制率
% if Tnum==1&&Tr.TrCir(Tnum)==0 %如果是第一圈的第一辆车直接将控制量设为0
%     uik=0;
% else
%     uik=Get_iterative(Tnum,Pnum,TimeSchedule,Tr,Plat,control);
% end
uik=Get_iterative(Tnum,Pnum,TimeSchedule,Tr,Plat,control);

%加入控制
% if error+uik<0
%     uik=-error;
% end
t2=clock;
uik=round(uik,-1);
arrivetime=TimeSchedule.OriTimeTable(Tnum,Tr.TrCir(Tnum)*Plat.N*2+2*(Tr.TrPass(Tnum))+1)+error+uik;%此处加入+uik为加入控制
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=ceil(beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.BeginTimeTable));
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=arrivetime;
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.TimeUkActal(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
TimeSchedule.solvetime=[TimeSchedule.solvetime etime(t2,t1)];