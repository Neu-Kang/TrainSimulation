function TimeSchedule=updataTimeTable3(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%���µ��ﳵ����ʱ�̱�����׷���Ŀ������Ŀ�꣨������ʱ�̱�����
if Tr.TrCir(Tnum)==0 %����ǵ�һȦ�ĵ�һ����ֱ�ӽ���������Ϊ0
    uik=0;
else
    if Pnum+Plat.N*Tr.TrCir(Tnum)==10&&Tnum==2
       z=1; 
    end
    if Tnum==1
        %Tnum1Ϊǰһ���� Tnum2Ϊǰ�ڶ�����
        Tnum1=Tr.M;
        Tnum2=Tr.M-1;
    elseif Tnum==2
        Tnum1=Tnum-1;
        Tnum2=Tr.M;
    else
        Tnum1=Tnum-1;
        Tnum2=Tnum-2;
    end
    if Tnum==1
        Pnum1=Pnum+1+Plat.N*(Tr.TrCir(Tnum)-1);
    else
        Pnum1=Pnum+1+Plat.N*Tr.TrCir(Tnum);
    end
    %���������ϵ�� 
    h=-(1-control.ck(Pnum))^2/(control.q+(1-control.ck(Pnum))^2);
    g=-control.q/(control.q+(1-control.ck(Pnum))^2);
    f=(control.ck(Pnum)*control.q)/(control.q+(1-control.ck(Pnum))^2);
    %�����������
    ui2k=TimeSchedule.TimeUk(Tnum1,Pnum1); %��һ�����Ŀ�����
    yik=TimeSchedule.TimeHError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)); %ʱ�������
    %����ǰһվ̨
    
    yi2k=TimeSchedule.TimeHError(Tnum1,Pnum1); %ǰһ��������һ��վ̨�����
    if yi2k==0 %�����δ��¼H�������Ԥ��
       yi2k=predictHError(Tnum1,Pnum1,TimeSchedule,Tr,Plat);
    end

    uik=(1+h)*ui2k+g*yik+f*yi2k;
end
%�������
begintime=ceil(TimeSchedule.BeginTimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)+uik);%�˴�����+uikΪ�������
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=ceil(arrivebybegin(Tr,Plat,Tnum,begintime,TimeSchedule.BeginTimeTable));
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=begintime;
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;