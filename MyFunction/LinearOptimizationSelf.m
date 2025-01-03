function [timetable]=LinearOptimizationSelf(Tnum,Pnum,delaytime,timetable,Tr,Plat)
tic
%% 参数初始化
% 初始值设置
Tmin=Tr.Tmin;  % 最小停靠站时间
TSmin=Tr.TSmin; % 交路站最小停站时间
DDH=Plat.ArriveH; % 到到间隔时间  
DAH=Plat.DepartArriveH; % 发到间隔时间
NaH=Plat.DepartInter; % 自然间隔时间
% 计划时刻表(划分成只有晚点之后的所有站台)
depart_plan=timetable.OriTimeTable(:,2:2:end); 
arrive_plan=timetable.OriTimeTable(:,1:2:end);
[train_num,plat_num]=size(depart_plan);
MinRuntime=createRuntimeYou(timetable.MinRunTime,plat_num);
% RoutingStation=findRoute(Pnum,plat_num,Plat);
% 预测晚点
delay=delaypredict(DAH,NaH,Tnum,Tr.M,delaytime,Tr,Plat);
Pnum=Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum);
% 决策变量定义
depart_decision = sdpvar(train_num,plat_num); % 发车时间 xij表示第i站第j个列车的发车时间
arrive_decision = sdpvar(train_num,plat_num);  % 到站时间 yij表示第i+1站第j个列车的到站时间
Passenger = sdpvar(train_num,plat_num);  % 乘客数量建模
%% 约束定义
% 发车时间约束 区间运行时间约束 发到时间约束 到到约束 发发约束 停站时间约束
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%发车时间约束%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 发车时间约束(列车大于晚点,只有第一站)
Restraint=[];
%晚点发生之前站台（与原有时刻表保持一致）
for i=1:train_num
    for j=1:Pnum-1
        depart_decision(i,j)=depart_plan(i,j);       
    end
end
for i=1:train_num
    for j=1:Pnum
        arrive_decision(i,j)=arrive_plan(i,j);
    end
end
for j=1:Plat.N
    arrive_decision(1,j)=arrive_plan(1,j);
    depart_decision(1,j)=depart_plan(1,j);
end

%晚点以及之后站台（带有约束）
%加入晚点
for i=1:train_num
    depart_decision(i,Pnum)=depart_plan(i,Pnum)+delay(i);
end
% % 发车、到站车站时间约束（大于计划时间）
% for i=1:train_num
%     for j=Pnum+1:plat_num
%         Restraint=[Restraint depart_decision(i,j)>=depart_plan(i,j)];
%         Restraint=[Restraint arrive_decision(i,j)>=arrive_plan(i,j)];
%     end
% end
% 区间最小运行时间约束
for i=1:train_num
   for j=1:plat_num-1
       Restraint=[Restraint arrive_decision(i,j+1)-depart_decision(i,j)>=MinRuntime(j)];
   end
end
% % 发到时间约束
% for i=1:train_num-1
%    for j=Pnum:plat_num-1
%        Restraint=[Restraint arrive_decision(i+1,j)-depart_decision(i,j)>=DAH];
%    end
% end
% % 乘客数量求解
for i=1:train_num
    if i==1
        for j=1:plat_num
            if j<=Plat.N
                Restraint=[Restraint Passenger(i,j)>=Plat.ckpoint*NaH-0.001];
                Restraint=[Restraint Passenger(i,j)<=Plat.ckpoint*NaH+0.001];
            else
                Restraint=[Restraint Passenger(i,j)>=Plat.ckpoint*(depart_decision(i,j)-depart_decision(train_num,j-Plat.N))-0.001];
                Restraint=[Restraint Passenger(i,j)<=Plat.ckpoint*(depart_decision(i,j)-depart_decision(train_num,j-Plat.N))+0.001];
            end
        end
    else
        for j=1:plat_num
            Restraint=[Restraint Passenger(i,j)<=Plat.ckpoint*(depart_decision(i,j)-depart_decision(i-1,j))+0.001];
            Restraint=[Restraint Passenger(i,j)>=Plat.ckpoint*(depart_decision(i,j)-depart_decision(i-1,j))-0.001];
        end           
    end     
end
% 停站时间约束
for i=1:train_num
   for j=Pnum+1:plat_num
       if mod(j,plat_num/2)==1 % 在交路站时间长
           Restraint=[Restraint depart_decision(i,j)>=TSmin+Plat.P*Passenger(i,j)+arrive_decision(i,j)-0.001];
           Restraint=[Restraint depart_decision(i,j)<=TSmin+Plat.P*Passenger(i,j)+arrive_decision(i,j)+0.001];
       else %在一般站
           Restraint=[Restraint depart_decision(i,j)<=Tmin+Plat.P*Passenger(i,j)+arrive_decision(i,j)+0.001];
           Restraint=[Restraint depart_decision(i,j)>=Tmin+Plat.P*Passenger(i,j)+arrive_decision(i,j)-0.001];
       end
   end
end


% % 发发时间约束
% for i=1:train_num-1
%    for j=Pnum+1:plat_num
%        Restraint=[Restraint depart_decision(i+1,j)-depart_decision(i,j)>=DDH];
%    end
% end

%到发约束
for i=1:train_num
   for j=2:plat_num
       Restraint=[Restraint depart_decision(i,j)>=arrive_decision(i,j)]       
   end
end

%% 求解目标函数配置
ops = sdpsettings('verbose',0,'solver','cplex');
DelayFa = sum(sum(abs(depart_decision-depart_plan))); % 发车晚点时间
% DelayDao = sum(sum(arrive_decision-arrive_plan)); %到站晚点时间
J = DelayFa;
saveampl(Restraint,J,'mymodel');
result = solvesdp(Restraint,J,ops);%默认求解目标函数最小值
if result.problem ~= 0 % problem =0 代表求解成功
    disp('求解出错');
end
%通过value函数获取决策变量最优解取值
depart_time=value(depart_decision);
arrive_time=value(arrive_decision);


%% 输出设置

%求解完整调度时刻表
timetableAfter=zeros(train_num,plat_num*2);
timetableAfter(:,1:2:end)=arrive_time;
timetableAfter(:,2:2:end)=depart_time;
% timetableAfter = round(timetableAfter);
timetable.TimeTable=timetableAfter(:,1:2:end);
timetable.TimePlan=timetableAfter;
timetable.RunTime=solveRunTime(timetableAfter); 
%计算时间输出
toc
   %% 实际运行图与初始计划调度运行图对比展示部分
figure('name','Train Time-Distance BeginDiagram');
set(gca,'FontSize',15);
set(gca,'XLim',[0 250]);
set(gca,'YLim',[0 166]);
set(gca,'YTick',[0 84 120 140 166]);% Y轴的记号点 
set(gca,'YTicklabel',{'1(BJN)','2(WQ)','3(TJ)','4(JLCB)','5(BH)'});% Y轴的记号
cm = colormap('Lines');
ArriveNum=getlength(timetable.TimePlan);
mkr=['o';'*';'.';'x';'s';'^'; 'v'; 'd';'>';'<';'p';'h';'+'];
xlabel('Time (min)'); ylabel('Station ID x(t)');
title('Train Time-Distance Diagram');
for i=1:Tr.M
    hold on;
    time=createTime(ArriveNum(i),Plat.N,Plat.PlatX);
    plot(timetable.TimePlan(i,1:length(time)),time,...
          'Color', cm(i,:),'LineWidth', 2);

    plot(timetable.OriTimeTable(i,1:length(time)),time,...
        'Color', cm(i,:).*[0.6 0.6 0.6], 'LineWidth',0.5, ...
        'LineStyle','-.', 'Marker', mkr(mod(i,13)+1));
end
disp(['运行时间: ',num2str(toc)]);

end