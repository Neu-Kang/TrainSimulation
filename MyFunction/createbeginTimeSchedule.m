
function y=createbeginTimeSchedule(H,M,N,inter,PlatN)
%函数功能：根据时间间隔创建车辆发车时刻表
for i=1:M
    for j=1:N
        if j==1
            y(i,j)=(j-1)*H+(i-1)*H;
        else
            if mod(j,PlatN)-1==-1
                y(i,j)=y(i,j-1)+inter(end-1);
            elseif mod(j,PlatN)-1==0
                y(i,j)=y(i,j-1)+inter(end);
            else                
                y(i,j)=y(i,j-1)+inter(mod(j,PlatN)-1);
            end
            
        end
    end
end