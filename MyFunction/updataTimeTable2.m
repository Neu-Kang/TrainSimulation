function TimeSchedule=updataTimeTable2(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%���µ��ﳵ����ʱ�̱�����׷���Ŀ������Ŀ�꣨������ʱ�̱�������
tic
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
uik=round(uik,-1);
%�������
arrivetime=TimeSchedule.OriTimeTable(Tnum,(Pnum+Plat.N*Tr.TrCir(Tnum))*2+1)+uik+TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum));%�˴�����+uikΪ�������
% if Tnum==1
% %     if  Tr.TrCir(Tnum)~=0
% %         if arrivetime<=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*(Tr.TrCir(Tnum)-1)+1)
% %             arrivetime=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*(Tr.TrCir(Tnum)-1)+1)+2;
% %         end
% %     end
% else
%     if arrivetime<=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*Tr.TrCir(Tnum)+1)
%         arrivetime=TimeSchedule.TimeTable(cirsub(Tnum,Tr.M),Pnum+Plat.N*Tr.TrCir(Tnum)+1)+2;
%     end
% end    
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=arrivetime;
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.TimeTable);
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
toc