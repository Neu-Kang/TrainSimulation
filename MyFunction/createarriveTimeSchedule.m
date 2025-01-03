function y=createarriveTimeSchedule(platN,x,stayFirst,M,N,num,stayH,stayHS,PlatN)
%函数功能：创建车辆到站时刻表
for i=1:M
    for j=1:N
        if j==1
            y(i,j)=0;
        else
            if (i==1)&&(j>=2)&&(j<=num+1) 
                if mod(j,platN/2)==1
                    y(i,j)=x(i,j)-stayHS;
                else
                    y(i,j)=x(i,j)-stayH;
                end

            else
                if mod(j,platN/2)==1
                    y(i,j)=x(i,j)-stayHS;
                else
                    y(i,j)=x(i,j)-stayH;
                end
            end
        end
    end
end
