function [time]=createRunTime(oritime)
%�������ܣ����ڴ��������г����е�ʱ����ʱ�̱�
%������Ƽ���ʱ�����Ͻ����޸�
for i=1:((size(oritime,2)-1)/2)
    time(:,i)=oritime(:,2*i+1)-oritime(:,2*i) ;
end