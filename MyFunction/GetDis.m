function y=GetDis(PlatX,Distance)
%�������ܣ�����վ̨λ�ü����վ̨�����
y(1)=PlatX(2)-0;
for i=2:length(PlatX)-1
    y(i)=PlatX(i+1)-PlatX(i);
end
y(length(PlatX))=Distance-PlatX(length(PlatX));
