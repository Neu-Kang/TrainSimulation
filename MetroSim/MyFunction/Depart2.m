function [Tr,TimeSchedule,Plat]=Depart2(num,TimeSchedule,Plat,Tr)
%�������ܣ�����Ƿ���з������������趨�������ٶȺ���,���ط����������л�ʱ��ͷ�������

if num~=0
    syms x;
    Tr.TrB(num)=true; %����
    Plat.PlatNum(1)=0 ;%��վ��������
    [Tr,TimeSchedule]=GetPlan(num,TimeSchedule,Plat,Tr); %�õ��µļƻ���
    
end