function judge=checksafeinterval(time,i,Tr,Plat)
%�������ܣ�����Ƿ���֮��������С����ʱ���࣬���û��������Сʱ��������
if time-Plat.timelast(Tr.TrState(i))>=Plat.Tinterval
    judge=1;
else
    judge=0;
end

