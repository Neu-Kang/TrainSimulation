 function [Tr,Plat,TimeSchedule,ylast,control]=checkout2(time,TimeSchedule,Plat,Tr,control,ylast)
%函数功能:检测是否有出站列车(满足最小行车间隔)，若有出站，并将站台和列车状态和站台人数置零,创建新的列车运行速度
for i=1:Tr.M
    if Tr.TrState(i)~=0 
        if (time-Tr.TrIn(i))>=Tr.TrStay(i)
            if checksafeinterval(time,i,Tr,Plat)
                Plat_all=Tr.TrCir(i)*Plat.N+Tr.TrPass(i);
                if Tr.TrPass(i)+Plat.N*Tr.TrCir(i)==66&&i==8&&control.sort==1
                    zzz=1;
                end
                if Plat_all==65&&i==8&&control.sort==1
                    zzz=1;
                end
                Plat.timelast(Tr.TrState(i))=time;
                Plat.TimeD(Tr.TrState(i))=time;
                TimeSchedule.TimeArrive(i,Tr.TrCir(i)*Plat.N*2+2*(Tr.TrPass(i)))=time;
                TimeSchedule.TimeBegin(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time;
                TimeSchedule.TimeError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.BeginTimeTable(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i));
                TimeSchedule.TimeYHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.ChangeOriTimeTable(i,Tr.TrCir(i)*Plat.N*2+Tr.TrPass(i)*2);
                if i==1 %第一圈列车一间隔误差为0，第二圈开始计数
                    if Tr.TrCir(i)==0
                        TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=0;
                    else               
                        if TimeSchedule.TimeArrive(Tr.M,((Tr.TrCir(i)-1)*Plat.N+Tr.TrPass(i))*2)==0
                            for j=1:15
                               if  TimeSchedule.TimeArrive(cirsub(i,Tr.M,j),((Tr.TrCir(i)-1)*Plat.N+Tr.TrPass(i))*2)~=0
                                   TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.TimeArrive(cirsub(i,Tr.M,j),((Tr.TrCir(i)-1)*Plat.N+Tr.TrPass(i))*2)-(j-1)*6-Plat.DepartInter;
                                   break
                               end
                            end
                        else
                            TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.TimeArrive(Tr.M,((Tr.TrCir(i)-1)*Plat.N+Tr.TrPass(i))*2)-Plat.DepartInter;
                        end
                    end
                else 
                    if TimeSchedule.TimeArrive(i-1,(Tr.TrCir(i)*Plat.N+Tr.TrPass(i))*2)==0
                        for j=1:15
                           if  TimeSchedule.TimeArrive(cirsub(i,Tr.M,j),(Tr.TrCir(i)*Plat.N+Tr.TrPass(i))*2)~=0
                               TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.TimeArrive(cirsub(i,Tr.M,j),(Tr.TrCir(i)*Plat.N+Tr.TrPass(i))*2)-j*6-Plat.DepartInter;
                               break
                           end
                        end
                    else
                        TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=time-TimeSchedule.TimeArrive(i-1,(Tr.TrCir(i)*Plat.N+Tr.TrPass(i))*2)-Plat.DepartInter;
                    end
                end
%                 if abs(TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i)))<=3.5
%                     TimeSchedule.TimeHError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=0;
%                 end
                if control.sort==1
                    TimeSchedule.TimePlan=[TimeSchedule.OriTimeTable TimeSchedule.TimePlan(:,end)];
                end
                if control.sort==2 %有时刻表控制
                    TimeSchedule=updataTimeTable2(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==3 %无时刻表控制
                    TimeSchedule=updataTimeTable3(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==4 %鲁棒模型预测控制
                   TimeSchedule=updataTimeTable4(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
%                   TimeSchedule=updataTimeTable4(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==5 %迭代学习模型预测控制
                    TimeSchedule=updataTimeTable5(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==6 %优化方法控制
%                     TimeSchedule=updataTimeTable6(i,Tr.TrPass(i),TimeSchedule,Tr,Plat);
                    if Tr.TrCir(i)==control.last+1
                        control.optimization=0;
                    end
                    if TimeSchedule.TimeError(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))>=2&&control.optimization==0
                        TimeSchedule=updataTimeTable6(i,Tr.TrPass(i),TimeSchedule,Tr,Plat);
                        control.optimization=1;
                        control.last=Tr.TrCir(i);
                    end
                end
                if control.sort==7 %人工调度
                    TimeSchedule=updataTimeTable7(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==8
                    TimeSchedule=updataTimeTable8(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==10 %行车间隔鲁棒模型预测控制
                    TimeSchedule=updataTimeTable10(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==6
%                     [Tr,TimeSchedule]=GetPlan3(TimeSchedule,time,i,Tr.TrState(i),Plat,Tr);
                    [Tr,TimeSchedule]=GetPlan2(TimeSchedule,time,i,Tr.TrState(i),TimeSchedule.TimeTable,Plat,Tr); %得到新的列车运行速度
                else
                    [Tr,TimeSchedule]=GetPlan2(TimeSchedule,time,i,Tr.TrState(i),TimeSchedule.TimeTable,Plat,Tr); %得到新的列车运行速度
                end
                
                Plat.PlatState(Tr.TrState(i))=0;
                Plat.PlatNum(Tr.TrState(i))=0;
                Tr.TrState(i)=0;
                Tr.TrIn(i)=0;
                Tr.TrStay(i)=0; 
%                 [ylast,ylast2,judge]=checkOver(Tr,ylast);6
%                 if judge==1
%                    disp('出现越行')
%                    ylast2
%                    ylast
%                 end
            end
        end
    end
end