function y=createarriveTimeSchedule(x,stayFirst,M,N,num,stayH,PlatN)
%�������ܣ�����������վʱ�̱�
for i=1:M
    for j=1:N
        if j==1
            y(i,j)=0;
        else
            if (i==1)&&(j>=2)&&(j<=num+1)       
                y(i,j)=x(i,j)-stayH;
            else
                y(i,j)=x(i,j)-stayH;
            end
        end
    end
end
