function TimeSchedule=updataTimeTable2(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%���µ��ﳵ����ʱ�̱�����׷���Ŀ������Ŀ�꣨������ʱ�̱�����
if Tnum==1&&Tr.TrCir(Tnum)==0 %����ǵ�һȦ�ĵ�һ����ֱ�ӽ���������Ϊ0
    uik=0;
else
    g=-(control.q+control.p)/((control.ck(Pnum)-1)^2+(control.p+control.q));
    f=(control.q+control.ck(Pnum)*control.p)/((control.ck(Pnum)-1)^2+(control.p+control.q));
    if Tnum-1==0
        if TimeSchedule.TimeError(Tr.M,Pnum+1+Plat.N*Tr.TrCir(Tnum))~=0%��������㷨����Ҫ��
             uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimeSchedule.TimeError(Tr.M,Pnum+1+Plat.N*Tr.TrCir(Tnum));
        else %Ϊ������Ҫ����Ԥ��
            uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimePredict(Tr.M,Pnum+1+Plat.N*Tr.TrCir(Tnum),TimeSchedule);
        end
    else
        if TimeSchedule.TimeError(Tnum-1,Pnum+1+Plat.N*Tr.TrCir(Tnum))~=0 %����������㷨����Ҫ��
            uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimeSchedule.TimeError(Tnum-1,Pnum+1+Plat.N*Tr.TrCir(Tnum));
        else %δ���������Ԥ�����
            uik=g*TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+f*TimePredict(Tnum-1,Pnum+1+Plat.N*Tr.TrCir(Tnum),TimeSchedule);
        end
    end
end
%�������
begintime=ceil(TimeSchedule.BeginTimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)+uik);%<bn,kmjoudlf.f l lr/;.eswz4ikyjn�˴�����+uikΪ�������
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=ceil(arrivebybegin(Tr,Plat,Tnum,begintime,TimeSchedule.BeginTimeTable));
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=begintime;
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;