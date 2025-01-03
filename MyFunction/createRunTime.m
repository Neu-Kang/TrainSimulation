function [time]=createRunTime(M,Npast,RunTime)
%函数功能：用于创建用于列车运行的时间间隔时刻表
%加入控制即在时间间隔上进行修改
for i=1:Npast
    z=mod(i,size(RunTime,2));
    if z==0
        time(:,i)=RunTime(end);
    else
        time(:,i)=RunTime(z);
    end
end
time=repmat(time,M,1);