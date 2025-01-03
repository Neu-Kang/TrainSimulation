function TimeSchedule=updataTimeTable10(Tnum,Pnum,TimeSchedule,Tr,Plat,control)

if Tnum==1&&Tr.TrCir(Tnum)==0
    return 
else
tic
%鲁棒模型预测控制
M=Tr.M; %16辆列车
N=Plat.N; %8个站台
error=TimeSchedule.TimeError(Tnum,Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum));
x_real=zeros(1,Tr.M);
t=1;
if Tr.TrCir(Tnum)==0
    Tk=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Tk=[Tk TimeSchedule.TimeBegin(trnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Tk=[Tk TimeSchedule.TimeBegin(trnum,plnum+Plat.N*Tr.TrCir(Tnum))];
        j=j+1;
    end
else
    Tk=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Tk=[Tk TimeSchedule.TimeBegin(trnum,Pnum+Plat.N*Tr.TrCir(Tnum))];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Tk=[Tk TimeSchedule.TimeBegin(trnum,plnum+Plat.N*Tr.TrCir(Tnum))];
        j=j+1;
    end        
end
% 前一时刻误差
if Tnum==1
    Tnum1=Tr.M;
    Tnum1Cir=Tr.TrCir(Tnum)-1;
else
    Tnum1=Tnum-1;
    Tnum1Cir=Tr.TrCir(Tnum);
end
if Tr.TrCir(Tnum1)==0
    Tk1=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum1-i+N-1,M);
        Tk1=[Tk1 TimeSchedule.TimeBegin(trnum,Pnum+Plat.N*Tr.TrCir(Tnum1)+1)];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum1-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Tk1=[Tk1 TimeSchedule.TimeBegin(trnum,plnum+Plat.N*Tr.TrCir(Tnum1))];
        j=j+1;
    end
else
    Tk1=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum1-i+N-1,M);
        Tk1=[Tk1 TimeSchedule.TimeBegin(trnum,Pnum+Plat.N*Tr.TrCir(Tnum1))];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum1-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Tk1=[Tk1 TimeSchedule.TimeBegin(trnum,plnum+Plat.N*Tr.TrCir(Tnum1))];
        j=j+1;
    end        
end

% for i=Tnum-2-(Tr.M-Plat.N-1):Tnum-2
%     num=mod2(i,Tr.M);
%     pnum=mod2(Pnum+1,Plat.N);
%     if TimeSchedule.TimeError(num,pnum+Plat.N*Tr.TrCir(num))~=0
%         error=TimeSchedule.TimeError(num,pnum+Plat.N*Tr.TrCir(num));
%     else
%         error=TimePredict(num,pnum+Plat.N*Tr.TrCir(num),TimeSchedule);
%     end
%     x_real(t)=error;
%     t=t+1;
% end
% 
% for i=1:Plat.N
%     num=mod2(Tnum-2+i,Tr.M);
%     pnum=mod2(Pnum+2-i,Plat.N);
%     if TimeSchedule.TimeError(num,pnum+Plat.N*Tr.TrCir(num))~=0
%         error=TimeSchedule.TimeError(num,pnum+Plat.N*Tr.TrCir(num));
%     else
%         error=TimePredict(num,pnum+Plat.N*Tr.TrCir(num),TimeSchedule);
%     end
%     x_real(t)=error;
%     t=t+1;
% end
uik1=TimeSchedule.TimeUk(Tnum1,Pnum+Plat.N*Tnum1Cir);
y_real=Tk-Tk1-TimeSchedule.H;
if y_real==0
    Auik=0;
else
    u_real=LMIsolve(Tr.M,Plat.N,y_real',control);
    Auik=u_real(end);
    uik=Auik+uik1;
end
uik=round(uik,-1);
arrivetime=TimeSchedule.TimeArrive(Tnum,Tr.TrCir(Tnum)*Plat.N*2+2*(Tr.TrPass(Tnum)))+error+uik;%此处加入+uik为加入控制
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.BeginTimeTable);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=arrivetime;
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
toc

end
end