function [time]=createRunTime(M,Npast,RunTime)
%�������ܣ����ڴ��������г����е�ʱ����ʱ�̱�
%������Ƽ���ʱ�����Ͻ����޸�
for i=1:Npast
    z=mod(i,size(RunTime,2));
    if z==0
        time(:,i)=RunTime(end);
    else
        time(:,i)=RunTime(z);
    end
end
time=repmat(time,M,1);