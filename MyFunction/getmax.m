function [data,order]=getmax(TrX,num,dis)
%�������ܣ������ڵ�һ���ĳ�����Ű������ɴ�С����
data=[];
order=[];
for i=2:length(num)
   if TrX(i)>dis
       data(length(data)+1)=TrX(i);
       order(length(order)+1)=num(i);
   end
end

for i=1:length(order)-1
   for j=i+1:length(order)
       if data(j-1)<data(j)
          z=data(j-1);
          data(j-1)=data(j);
          data(j-1)=z;
          z=order(j-1);
          order(j-1)=order(j);
          order(j)=z;
       end
   end
end