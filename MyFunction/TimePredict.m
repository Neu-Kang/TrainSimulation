function predict=TimePredict(Tnum,Pnum,TimeSchedule)
%�������ܣ�����δ��վ������ģ��Ԥ�⣨Ԥ�⵽վ����
predict=0;
for i=Pnum:-1:1
    if TimeSchedule.TimeError(Tnum,i)~=0 %�ҵ�������Ǹ�վ����ǰԤ��һվ
        predict=TimeSchedule.TimeError(Tnum,i)+TimeSchedule.TimeUk(Tnum,i); 
        break;
    end
    predict=0;
end
% predict