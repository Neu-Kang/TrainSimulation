function TimeSchedule=updataTimeTable5(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%���µ��ﳵ����ʱ�̱�����׷���Ŀ������Ŀ�꣨������ʱ�̱�����
%���������
error=TimeSchedule.TimeError(Tnum,Tr.TrCir(Tnum)*Plat.N+Tr.TrPass(Tnum));
t1=clock;
if Tnum==1&& Pnum==3&&Tr.TrCir(Tnum)==1
    z=11;
end
%���������
% if Tnum==1&&Tr.TrCir(Tnum)==0 %����ǵ�һȦ�ĵ�һ����ֱ�ӽ���������Ϊ0
%     uik=0;
% else
%     uik=Get_iterative(Tnum,Pnum,TimeSchedule,Tr,Plat,control);
% end
uik=Get_iterative(Tnum,Pnum,TimeSchedule,Tr,Plat,control);

%�������
% if error+uik<0
%     uik=-error;
% end
t2=clock;
uik=round(uik,-1);
arrivetime=TimeSchedule.OriTimeTable(Tnum,Tr.TrCir(Tnum)*Plat.N*2+2*(Tr.TrPass(Tnum))+1)+error+uik;%�˴�����+uikΪ�������
TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)=ceil(beginbyarrive(Tr,Plat,Tnum,arrivetime,TimeSchedule.BeginTimeTable));
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)+Plat.N*2*Tr.TrCir(Tnum))=TimeSchedule.TimeTable(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1);
TimeSchedule.TimePlan(Tnum,2*(Pnum+1)-1+Plat.N*2*Tr.TrCir(Tnum))=arrivetime;
TimeSchedule.TimeUk(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.TimeUkActal(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=uik;
TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))=TimeSchedule.RunTime(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum))+uik;
TimeSchedule.solvetime=[TimeSchedule.solvetime etime(t2,t1)];