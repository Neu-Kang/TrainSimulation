function [Tr,TimeSchedule]=Depart(num,TimeSchedule,Plat,Tr)
%�������ܣ�����Ƿ���з������������趨�������ٶȺ���,���ط����������л�ʱ��ͷ�������

if num~=0
    syms x;
    Tr.TrB(num)=true; %����
    Tr=GetPlan(num,TimeSchedule,Plat,Tr); %�õ��µļƻ���
    TimeSchedule=updataTimePlan2(num,Tr,TimeSchedule,Plat);
    
end