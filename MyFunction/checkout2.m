 function [Tr,Plat,TimeSchedule,ylast,control]=checkout2(time,TimeSchedule,Plat,Tr,control,ylast)
%��������:����Ƿ��г�վ�г�(������С�г����)�����г�վ������վ̨���г�״̬��վ̨��������,�����µ��г������ٶ�
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
                if i==1 %��һȦ�г�һ������Ϊ0���ڶ�Ȧ��ʼ����
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
                if control.sort==2 %��ʱ�̱����
                    TimeSchedule=updataTimeTable2(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==3 %��ʱ�̱����
                    TimeSchedule=updataTimeTable3(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==4 %³��ģ��Ԥ�����
                   TimeSchedule=updataTimeTable4(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
%                   TimeSchedule=updataTimeTable4(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==5 %����ѧϰģ��Ԥ�����
                    TimeSchedule=updataTimeTable5(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==6 %�Ż���������
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
                if control.sort==7 %�˹�����
                    TimeSchedule=updataTimeTable7(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==8
                    TimeSchedule=updataTimeTable8(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==10 %�г����³��ģ��Ԥ�����
                    TimeSchedule=updataTimeTable10(i,Tr.TrPass(i),TimeSchedule,Tr,Plat,control);
                end
                if control.sort==6
%                     [Tr,TimeSchedule]=GetPlan3(TimeSchedule,time,i,Tr.TrState(i),Plat,Tr);
                    [Tr,TimeSchedule]=GetPlan2(TimeSchedule,time,i,Tr.TrState(i),TimeSchedule.TimeTable,Plat,Tr); %�õ��µ��г������ٶ�
                else
                    [Tr,TimeSchedule]=GetPlan2(TimeSchedule,time,i,Tr.TrState(i),TimeSchedule.TimeTable,Plat,Tr); %�õ��µ��г������ٶ�
                end
                
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