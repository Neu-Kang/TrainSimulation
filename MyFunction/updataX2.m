function [Tr,Plat,TimeSchedule]=updataX2(time,k,TimeSchedule,Tr,Plat,control,noise)
%更新车辆的位置，包括判断是否到达车站，是否已到达总里程数
time=time/k;
for i=1:Tr.M
    %更新位置
    if Tr.TrState(i)~=0
        continue
    end
    Tr.TrX(i)=Tr.TrV(i)/k+Tr.TrX(i);
    if Tr.Staytime(i)>0
        Tr.TrX(i)=Tr.TrX(i)-Tr.TrV(i)/k;
        Tr.Staytime(i)=Tr.Staytime(i)-1;
    end
    %避免越行
%     ibef1=Get_Tr(i,Tr.M,1);
%     ibef2=Get_Tr(i,Tr.M,2);
%     if i==1||i==2
%         if Tr.TrCir(i)~=0
%            if Tr.TrX(i)>=Tr.TrX(ibef1)&&Tr.TrX(i)<=Tr.TrX(ibef2)
%                Tr.TrX(i)=Tr.TrX(ibef1)-1;
%            end
%         end
%     else
%         if Tr.TrX(i)>=Tr.TrX(ibef1)&&Tr.TrX(i)<=Tr.TrX(ibef2)&&Tr.TrX(i)~=0&&~ismember(Tr.TrX(i),Plat.PlatX)
%                Tr.TrX(i)=Tr.TrX(ibef1)-Tr.TrV(i)/k;
%                Tr.Staytime(i)=Tr.TrAT(i)-Tr.TrAT(ibef1)+1;
%                if Tr.Staytime(i)<0
%                    Tr.Staytime(i)=0;
%                end
%                Tr.TrCh(1,i)=Tr.TrCh(1,i)+ Tr.Staytime(i);
%                Tr.TrCh(2,i)=Tr.TrCh(2,i)+ Tr.Staytime(i);
%                Tr.TrAT(1,i)=Tr.TrAT(1,i)+ Tr.Staytime(i);
%         end
%     end
    if Tr.TrX(i)~=0
        Tr.first(i)=1;
    end
    if Tr.TrV(i)==0&&Tr.TrB(i)==true&&Tr.first(i)==1&&Tr.TrState(i)==0
        Tr.TrX(i)=getNearone(i,Tr,Plat);
    end
    
    %判断是否到达终点，若到达更新经过车站
    if Tr.TrX(i)>=Plat.Distance&&Tr.TrPass(i)==Plat.N 
        
        if  (time-Plat.TimeD(1))<Plat.DepartArriveH || Plat.PlatState(1)~=0 || Plat.TrWait(1)~=i ||time<TimeSchedule.TimePlan(i,2*(Tr.TrPass(i)+1)+Plat.N*2*Tr.TrCir(i)-1)
            if i==1
                df=0;
            end
            Tr.TrV(i)=0;  %将速度置为0
            Tr.TrFuc(1:4,i)=0;  %将车辆加速度置为0
            Plat.TrWait=Get_wait(Plat.TrWait,i);
        else
            Plat.TrWait=Delete_wait(Plat.TrWait);
            Tr.TrX(i)=0;
            Tr.TrPass(i)=1;
            Tr.TrCir(i)=Tr.TrCir(i)+1;
            [ptime,Tr]=putnoise(i,Tr.TrPass(i)+Plat.N*Tr.TrCir(i),k,time,Tr,Plat,noise);
            TimeSchedule.TimeTable(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=ptime;
            TimeSchedule.TimeArrive(i,Tr.TrCir(i)*Plat.N*2+2*Tr.TrPass(i)-1)=ptime;
            Tr.TrState(i)=1;
            Tr.TrIn(i)=ptime;
            Tr.TrV(i)=0;  %将速度置为0
            Tr.TrFuc(1:4,i)=0;  %将车辆加速度置为0
            Plat.TimeA(Tr.TrPass(i))=ptime;
            %清空乘客，设定停站时间
            t1=Plat.TimeA(Tr.TrPass(i))-Plat.TimeD(Tr.TrPass(i));
            t2=(t1+Plat.TSmin)/(1-Plat.a*Plat.ck(Tr.TrPass(i)));
            Tr.TrStay(i)=t2-t1;
%             if i==1&&Tr.TrCir(1)==0
%                 Tr.TrStay(i)=TimeSchedule.BeginTimeTable(i,Tr.TrPass(i))-TimeSchedule.TimeTable(i,Tr.TrPass(i));
%             else
%                 t1=Plat.TimeA(Tr.TrPass(i))-Plat.TimeD(Tr.TrPass(i));
%                 t2=(t1+Plat.TSmin)/(1-Plat.a*Plat.ck(Tr.TrPass(i)));
%                 Tr.TrStay(i)=t2-t1;
% %                 if Plat.PlatState(Tr.TrPass(i))~=0
% %                     Tr.TrStay(i)=(Plat.ck(Tr.TrPass(i))*Plat.P*(ptime-Tr.TrIn(cirsub(i,Plat.N))-Tr.TrStay(cirsub(i,Plat.N)))+Tr.TSmin) ...
% %                         /(1-Plat.ck(Tr.TrPass(i))*Plat.P);
% %                 else
% %                     Tr.TrStay(i)=(Plat.PlatNum(Tr.TrState(i))*Plat.P+Tr.TSmin)/(1-Plat.ck(Tr.TrPass(i))*Plat.P);
% %                 end 
%             end
            Tr.TrStay(i)=putnoise2(i,Tr.TrPass(i),k,Tr.TrStay(i),Tr,Plat,noise);
            
            Plat.PlatState(1)=0;
            continue;
         end
    end
    if Tr.TrPass(i)~=Plat.N     
        if Tr.TrX(i)>=Plat.PlatX(Tr.TrPass(i)+1)
            if (Plat.PlatState(Tr.TrPass(i)+1)~=0 && Tr.TrPass(i)~=Plat.N) || time-Plat.TimeD(Tr.TrPass(i)+1)<Plat.DepartArriveH || Tr.TrX(i)==Tr.TrX(cirsub(i,Tr.M))||time<TimeSchedule.TimePlan(i,2*(Tr.TrPass(i)+1)+Plat.N*2*Tr.TrCir(i)-1)
                Tr.TrV(i)=0;  %将速度置为0
                Tr.TrFuc(1:4,i)=0;  %将车辆加速度置为0
            else
                
                Tr.TrX(i)=Plat.PlatX(Tr.TrPass(i)+1);
                Tr.TrPass(i)=Tr.TrPass(i)+1;               
                [ptime,Tr]=putnoise(i,Tr.TrPass(i)+Plat.N*Tr.TrCir(i),k,time,Tr,Plat,noise);
                TimeSchedule.TimeTable(i,Tr.TrCir(i)*Plat.N+Tr.TrPass(i))=ptime;
                TimeSchedule.TimeArrive(i,Tr.TrCir(i)*Plat.N*2+2*Tr.TrPass(i)-1)=ptime;
                Tr.TrState(i)=Tr.TrPass(i);
                Tr.TrIn(i)=ptime;
                Plat.TimeA(Tr.TrPass(i))=ptime; 
                Tr.TrV(i)=0;   %将速度置为0
                Tr.TrFuc(1:4,i)=0;   %将车辆加速度置为0 
                %设定停站时间(包括预测在站台期间来的乘客)
                if i==1&&Tr.TrPass(i)==8
                   z=1; 
                end
                if Tr.TrPass(i)== Plat.N/2+1
                    t1=Plat.TimeA(Tr.TrPass(i))-Plat.TimeD(Tr.TrPass(i));
                    t2=(t1+Plat.TSmin)/(1-Plat.a*Plat.ck(Tr.TrPass(i)));
                    Tr.TrStay(i)=t2-t1;  
                else
                    t1=Plat.TimeA(Tr.TrPass(i))-Plat.TimeD(Tr.TrPass(i));
                    t2=(t1+Plat.Tmin)/(1-Plat.a*Plat.ck(Tr.TrPass(i)));
                    Tr.TrStay(i)=t2-t1;
                end
               

%                 if i==1&&Tr.TrCir(1)==0
%                     Tr.TrStay(i)=TimeSchedule.BeginTimeTable(i,Tr.TrPass(i))-TimeSchedule.TimeTable(i,Tr.TrPass(i));
%                 end
                Tr.TrStay(i)=putnoise2(i,Tr.TrPass(i),k,Tr.TrStay(i),Tr,Plat,noise);

                Plat.PlatState(Tr.TrPass(i))=0;
            end
        end
    end
end
