function TimeScheduleAfter=updataTimeTable6(Tnum,Pnum,TimeSchedule,Tr,Plat)
% Error=TimeSchedule.TimeYHError(Tnum,Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum));
% if Error>=10%如果满足算法计算要求
%     TimeSchedule.ChangeOriTimeTable=LinearOptimization(Tnum,Pnum,Error,TimeSchedule.ChangeOriTimeTable);
%     TimeSchedule.Runtime=createRunTime(TimeSchedule.ChangeOriTimeTable);
% end

delaytime=TimeSchedule.TimeError(Tnum,Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum));
if delaytime>0.5
    TimeScheduleAfter=LinearOptimizationSelf(Tnum,Pnum,delaytime,TimeSchedule,Tr,Plat);
else
    TimeScheduleAfter=TimeSchedule;
end
end