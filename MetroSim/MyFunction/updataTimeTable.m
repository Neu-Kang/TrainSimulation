function TimeSchedule=updataTimeTable(Tnum,Pnum,TimeSchedule,Tr,control)
%���µ��ﳵ����ʱ�̱�����׷���Ŀ������Ŀ��
g=(control.q+2*control.p)/((control.ck(Pnum)-1)*(control.p+control.q+1));
f=control.ck(Pnum)*(control.p+control.q)/((control.ck(Pnum)-1)*(control.p+control.q+1));
if Tnum-1==0
    uik=-g*TimeSchedule.TimeError(Tnum,Pnum+3*Tr.TrCir(Tnum));
else
    if TimeSchedule.TimeError(Tnum-1,Pnum+1+3*Tr.TrCir(Tnum))~=0 %����������㷨����Ҫ��
        uik=-g*TimeSchedule.TimeError(Tnum,Pnum+3*Tr.TrCir(Tnum))+f*TimeSchedule.TimeError(Tnum-1,Pnum+1+3*Tr.TrCir(Tnum));
    else %δ���������Ԥ�����
        uik=-g*TimeSchedule.TimeError(Tnum,Pnum+3*Tr.TrCir(Tnum))+f*TimePredict(Tnum-1,Pnum+1+3*Tr.TrCir(Tnum),TimeSchedule);
    end
end
TimeSchedule.TimeTable(Tnum,Pnum+3*Tr.TrCir(Tnum)+1)=TimeSchedule.TimeTable(Tnum,Pnum+3*Tr.TrCir(Tnum)+1)+uik;
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+6*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+3*Tr.TrCir(Tnum)+1);
TimeSchedule.TimeUk(Tnum,Pnum+Tr.TrCir(Tnum))=uik;
    