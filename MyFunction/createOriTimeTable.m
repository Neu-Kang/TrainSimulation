function OriTimeTable=createOriTimeTable(M,platN,begintime,steps,stayH,stayHS)
%�������ܣ����ݰ�ȫʱ�����͵�һ��������ʱ��
for i=1:M
    for j=1:steps/2
        if j==1
            OriTimeTable(i,2*j-1)=0;
        else
            if mod(j,platN/2)==1
                OriTimeTable(i,2*j-1)=begintime(i,j)-stayHS;
            else
                OriTimeTable(i,2*j-1)=begintime(i,j)-stayH;
            end
            
        end
        OriTimeTable(i,2*j)=begintime(i,j);
    end
end
