function TimeSchedule=updataTimeTable4(Tnum,Pnum,TimeSchedule,Tr,Plat,control)

tic
%鲁棒模型预测控制
M=Tr.M; %16辆列车
N=Plat.N; %8个站台
error=TimeSchedule.TimeError(Tnum,Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum));
x_real=zeros(1,Tr.M);
t=1;
if Tr.TrCir(Tnum)==0
    Xk=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Xk=[Xk TimeSchedule.TimeError(trnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Xk=[Xk TimeSchedule.TimeError(trnum,plnum+Plat.N*Tr.TrCir(Tnum))];
        j=j+1;
    end
else
    Xk=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Xk=[Xk TimeSchedule.TimeError(trnum,Pnum+Plat.N*Tr.TrCir(Tnum))];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Xk=[Xk TimeSchedule.TimeError(trnum,plnum+Plat.N*Tr.TrCir(Tnum))];
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
x_real=Xk';
if x_real(10)==0
    uik=0;
else
    u_real=LMIsolve(Tr.M,Plat.N,x_real,control);
    uik=u_real(2);
    if uik>=0
       uik=0; 
    end
    uik=round(uik,-1);
end
arrivetime=ceil(TimeSchedule.TimeArrive(Tnum,Tr.TrCir(Tnum)*Plat.N*2+2*(Tr.TrPass(Tnum)))+error+uik);%此处加入+uik为加入控制
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=ceil(beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.BeginTimeTable));
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=arrivetime;
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
toc

end