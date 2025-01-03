function TimeSchedule=updataTimeTable8(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%人工调度策略尽量恢复当前列车晚点
if mod(Pnum,Plat.N/2)==1
    error=TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum));
    if error~=0
        fff=0;
    end
    if TimeSchedule.RunTime(Tnum,Pnum)-error<TimeSchedule.MinRunTime(Pnum)
       uik=-(TimeSchedule.RunTime(Tnum,Pnum)-TimeSchedule.MinRunTime(Pnum));

    else
       uik=-error;
    end
    arrivetime=ceil(TimeSchedule.OriTimeTable(Tnum,(Pnum+Plat.N*Tr.TrCir(Tnum))*2+1)+uik+TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)));%此处加入+uik为加入控制

    TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=arrivetime;
    TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
    TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=ceil(beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.TimeTable));
    TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
    TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
end
end