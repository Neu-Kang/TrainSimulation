function [ptime,Tr]=putnoise(i,j,k,time,Tr,noise)
%�������ܣ�������ʱ����Ӹ���
ptime=time;
if i==2&&j==10
    ptime=time+15;
    fprintf("�г�%d��վ̨%d������Ϊ%f",i,j,8);
end
end

% if Tr.noise(i)==1 %�Ѵﵽ�����г�
%     if ~(i==1&&Tr.TrCir(1)==0)
%         fprintf("�г�%d��վ̨%d������Ϊ%f",i,j,noise(time*k));
% %         time=time+3.5;
%         Tr.noise(i)=-1; %�ڽ���һ�θ���֮����ͣ����
%     else
%         time=time;
%     end
% else
%     time=time;
% end
% if Tr.noise(i)~=-1 %���������
%     Tr.noise(i)=ciradd(Tr.noise(i),Tr.noisetime);
% end
% time=round(time);