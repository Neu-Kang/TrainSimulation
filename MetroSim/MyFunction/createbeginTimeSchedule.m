function y=createbeginTimeSchedule(H,M,N,inter,PlatN)
%�������ܣ�����ʱ����������������ʱ�̱�
for i=1:M
    for j=1:N
        if j==1
            y(i,j)=(j-1)*H+(i-1)*H;
        else
            y(i,j)=y(i,j-1)+inter(mod(j,PlatN)+1);
        end
    end
end