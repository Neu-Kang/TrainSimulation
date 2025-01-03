function TimeSchedule=updataTimeTable2(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%更新到达车辆的时刻表，函数追求的目标是与目标（带名义时刻表场景）】
tic
g=-(control.q+control.p)/((control.ck(Pnum)-1)^2+(control.p+control.q));
f=(control.q+control.ck(Pnum)*control.p)/((control.ck(Pnum)-1)^2+(control.p+control.q));
if Tnum-1==0
    if TimeSchedule.TimeError(Tr.M,Pnum+1+Plat.N*Tr.TrCir(Tnum))~=0%如果满足算法计算要求
         uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimeSchedule.TimeError(Tr.M,Pnum+1+Plat.N*Tr.TrCir(Tnum));
    else %为满足需要进行预测
        uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimePredict(Tr.M,Pnum+1+Plat.N*Tr.TrCir(Tnum),TimeSchedule);
    end
else
    if TimeSchedule.TimeError(Tnum-1,Pnum+1+Plat.N*Tr.TrCir(Tnum))~=0 %如果以满足算法计算要求
        uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimeSchedule.TimeError(Tnum-1,Pnum+1+Plat.N*Tr.TrCir(Tnum));
    else %未满足需进行预测误差
        uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimePredict(Tnum-1,Pnum+1+Plat.N*Tr.TrCir(Tnum),TimeSchedule);
    end
end
uik=round(uik,-1);
%加入控制
arrivetime=TimeSchedule.OriTimeTable(Tnum,(Pnum+Plat.N*Tr.TrCir(Tnum))*2+1)+uik+TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum));%此处加入+uik为加入控制
% if Tnum==1
% %     if  Tr.TrCir(Tnum)~=0
% %         if arrivetime<=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*(Tr.TrCir(Tnum)-1)+1)
% %             arrivetime=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*(Tr.TrCir(Tnum)-1)+1)+2;
% %         end
% %     end
% else
%     if arrivetime<=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*Tr.TrCir(Tnum)+1)
%         arrivetime=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*Tr.TrCir(Tnum)+1)+2;
%     end
% end    
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=arrivetime;
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.TimeTable);
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
toc