function OriTimeTable=createOriTimeTable(M,begintime,steps,stayH)
%�������ܣ����ݰ�ȫʱ�����͵�һ��������ʱ��
for i=1:M
    for j=1:steps/2
        if j==1
            OriTimeTable(i,2*j-1)=0;
        else
            OriTimeTable(i,2*j-1)=begintime(i,j)-stayH;
        end
        OriTimeTable(i,2*j)=begintime(i,j);
    end
end
