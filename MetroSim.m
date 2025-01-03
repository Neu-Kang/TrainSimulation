%{
���ߣ����ѿ�
���빦�ܣ�ʵ��λ�ô����Ļ��ߵ������ܷ���
�ο����ģ�Traffic Modeling and State Feedback Control for Metro Lines 
%}

%{
ʱ�䣺2021.4.19 22.04 
���޸����ݣ�����ƽ̨�ȴ�ʱ�䣬updataX��10��һ������վ���������� (�ѽ��)
ʱ�䣺2021.4.20 15.34
���޸����ݣ������г��ȳ��ȴ����޸Ŀ��ܵĵ�500ֹͣ���⣬���ٳ˿͵�Ӱ�죨�ѽ����
ʱ�䣺2021.4.21  9.45
���޸����ݣ�������403ʱ��ʱ���г���վ���ٳ�վ�����ҽ�վ�����СΪ1500���ѽ����
ʱ�䣺2021 4.21  16.02
���޸����ݣ����ӵ�վʱ����������ʱ�̱��ѽ����
ʱ�䣺2021 4.21  19.22
���޸����ݣ�����Ԥ��ʱ����������ʱ�̱Ƚϱ���������Ŷ���ѯ��ʦ�֣��ѽ����
ʱ�䣺2021 4.21  23.01
���޸����ݣ��ڶ�δ������ʱ�̱���е���ʱ����������������ɣ�updataTimeTable������ʽ�����⣨�ѽ����
ans =

     0   101   201   301   401   504   600   700


ans =

     0   101   201   301   401   504   799   700
ʱ�䣺2021 4.22
���޸����ݣ�Ԥ��ʱ�̱����Ԥ�⵽��ʱ���ڷ���ʱ��֮�������ʵ�������������Сͣ��ʱ��,�޸ļƻ���վʱ�䣬������ţ�δ������ţ�
ʱ�䣺2021.4.29
���޸����ݣ�ʱ�̱�����ڵ���վ���������⣬���ǿ�����˳���λ�õ�ת������
ʱ�䣺2021.5.10
���޸����ݣ����ӳ���֮�䰲ȫ�������ƣ�����ƴﵽ��ȫ����������У������������й����У�������֤���(�ѽ��)
ʱ�䣺2021.5.11
���޸����ݣ����ӳ�����ʼ����ͼ���޸�Ԥ�ⷢ�������޸��������ʱ��
ʱ�䣺2021.5.17
���޸����ݣ�ȡ���г��䰲ȫ���룬���Գ�վ����������㣩���޸�Ϊ�ı䷢��ʱ�䡣
˼·��
1.��ôͨ������ʱ��ȷ�������ٶ�
2.��ôԤ�ⷢ��ʱ��
%}
clc
clear all;
%% ������ʼ������

%ϵͳ�ı�����ʼ��,3��վ̨��5������
%�����Լ�ʹ����������޸�
%��ģ��ΪM>N�������
N=3;  %��վ���� 
M=5;  %�г����� 
P=0.01;  %�趨��վ������ͣ��ʱ���ϵ 
L=10000; %���沽��
k=1; %���沽������֮������
aTr=5;  %�г��̶����ٶ�
Tmin=5; %�г���վ̨ͣ����Сʱ��
noisetime=7; %�趨�г��յ����ż�վ�������һ�θ���
noisenum=1; %�趨�������յ�����Ŷ�
fisttrainT=100; %��һ���г�����ʱ��
internal=0; %��ģ���г�����ȫ���Ϊ0�����ͨ����վ����������
ckspeed=2; %�˿������ٶ�
%��վ��ʼ��
Plat.N=N; %��վ����
Plat.P=P; %�趨��վ������ͣ��ʱ���ϵ
Plat.Distance=1600; %�����ܾ��� (��Ҫ�Լ�����)
Plat.PlatX=[1600 500 1100]; %��վλ�� (��Ҫ�Լ�����)
Plat.ck=ckspeed*ones(1,Plat.N); %ÿ����վ�ĳ˿������ٶ�(��Ҫ�Լ�����)
Plat.PlatState=zeros(1,N); %��վ״̬���Ƿ��г���վ̨,������Ϊ�����кţ�������Ϊ0��
Plat.PlatNum=zeros(1,N); %��վ�ȴ�����
Plat.PlatDis=GetDis(Plat.PlatX,Plat.Distance);%��վ�����
Plat.timelast=zeros(1,N); %��վ��һ�����뿪ʱ��
Plat.Tinterval=internal;%����֮�䷢����ȫ���

%�����ʲ������
control.p=5; %ʱ�����������ϵ��
control.q=6; %���ȴ�С������ϵ��
control.ck=Plat.ck; %ÿ����վ�ĳ˿������ٶ�

%����������ʼ��
Tr.M=M;
Tr.Tmin=Tmin; %����ͣ�����ʱ��
Tr.TraTr=aTr; %�������ٶ�
Tr.TrX=zeros(1,M);%��������λ��
Tr.TrV=zeros(1,M);%������ǰ�ٶ�
Tr.TrA=zeros(1,M);%������ǰ���ٶ�
Tr.TrB=false(1,M);%�����Ƿ񷢳�
Tr.TrFuc=zeros(4,M);%�������еĺ��� y=a1x;y=k;y=a2x+b2;
Tr.TrCh=zeros(2,M);%�������١����١������л�ʱ��
Tr.TrBT=zeros(1,M);%�����ķ���ʱ��,�����ǰһ����վ
Tr.TrPass=ones(1,M);%������������һ����վ����δ����Ϊ0
Tr.TrCir=zeros(1,M);%��������Ȧ��
Tr.TrState=zeros(1,M);%����״̬�Ƿ�����վ��1Ϊ��վ�ڣ�0Ϊ��վ��
Tr.TrStay=zeros(1,M);%������վ��ͣ��ʱ��
Tr.TrIn=zeros(1,M);%������վʱ�䣬��δ��վΪ0
Tr.first=zeros(1,M);%�����Ƿ���뿪��վ
Tr.noisetime=noisetime;%�趨�յ����ż�վ�������һ�θ���
Tr.noise=setnoisenum(noisenum,Tr.M);%�����Ƿ���и���(-1Ϊ���Ӹ��ţ�0Ϊ�������)

%ʱ�̱��ʼ��
Steps=L/k;
TimeSchedule.TimeTable=[
0:100:100*Steps;  %0 100 + 20 120 100 220 + 23 243   L 500  100   
10:100:10+100*Steps;
20:100:20+100*Steps;
30:100:30+100*Steps;
40:100:40+100*Steps;
];%����ȷ���г���ÿ�ε��ٶ�,һ��Ϊһ���г�
TimeSchedule.OriTimeTable=createOriTimeTable(Plat.Tinterval,fisttrainT,Steps,Plat,Tr); %�����ƻ�����ͼ
TimeSchedule.TimeArrive=[zeros(5,1),TimeSchedule.TimeTable(:,1),zeros(M,Steps-1)];%��¼�г�ʵ�ʵ���
TimeSchedule.TimePlan=[zeros(5,1),TimeSchedule.TimeTable(:,1:2),zeros(M,Steps-2)];%��¼�г�Ԥ�Ƶ���ʱ��
TimeSchedule.TimeError=zeros(M,Steps);%[TimeSchedule.TimeTable(:,1),zeros(M,Steps-1)];%��¼ʱ�����
TimeSchedule.TimeUk=zeros(M,Steps);%��¼����ʱ��

Tr.TrFuc(1,:)=aTr;
Tr.TrFuc(3,:)=-aTr;%�趨�г����ٶ� 
H=100; %����֮�����İ�ȫ����
T=10;  %��������֮������ʱ��

%����������
noise=abs(sqrt(10)*randn(1,L));%����������,��ֵΪ0,����Ϊ10
%% ����ģ�����

for i=1:L
    time(i)=i;
    if i==107
        y=1;
    end
    Plat=updataPassenger(Plat); %���³�վ����
    [Tr,Plat,TimeSchedule]=updataX(i,k,TimeSchedule,Tr,Plat,control,noise); %�����г���ǰλ��
    if checkallout(Tr) %��ʼ��վ��������
        num=checkBegin(k*i,T,Tr); %��ȡ��վ�г�
        [Tr,TimeSchedule]=Depart(num,TimeSchedule,Plat,Tr); %���з���
    end
    [Tr,Plat,TimeSchedule]=checkout(k*i,TimeSchedule,Plat,Tr);
    Tr=getspeed(k*i,Tr);
    
end


%% �����г�����չʾ����չʾ����

ArriveNum=getlength(TimeSchedule.TimeArrive);
for i=1:M
    figure(i);
    time=createTime(ArriveNum(i));
    plot(TimeSchedule.TimeArrive(i,1:length(time)),time,'r-');
    hold on
    plot(TimeSchedule.TimePlan(i,1:length(time)),time,'g-');
    xlabel('t'); ylabel('x(t)');    
end
%% ʵ������ͼ��ʵʱ��������ͼ�Ա�չʾ����

    cm = colormap('Lines');
    mkr=['o';'*';'.';'x';'s';'^'; 'v'; 'd';'>';'<';'p';'h';'|';'-'];
    figure('name','Train Time-Distance Diagram');
    xlabel('Time (s)'); ylabel('Location x(t)');
    title('Train Time-Distance Diagram');
for i=1:M
    hold on;
    time=createTime(ArriveNum(i));
    plot(TimeSchedule.TimeArrive(i,1:length(time)),time,...
          'Color', cm(i,:),'LineWidth', 2);
    
    plot(TimeSchedule.TimePlan(i,1:length(time)),time,...
        'Color', cm(i,:).*[0.8, 0.8, 0.8], 'LineWidth', 1, ...
        'LineStyle','--', 'Marker', mkr(mod(i,14)+1));
end

%% ʵ������ͼ���ʼ�ƻ���������ͼ�Ա�չʾ����
    figure('name','Train Time-Distance BeginDiagram');
    xlabel('Time (s)'); ylabel('Location x(t)');
    title('Train Time-Distance Diagram');
for i=1:M
    hold on;
    time=createTime(ArriveNum(i));
    plot(TimeSchedule.TimeArrive(i,1:length(time)),time,...
          'Color', cm(i,:),'LineWidth', 2);
    
    plot(TimeSchedule.OriTimeTable(i,1:length(time)),time,...
        'Color', cm(i,:).*[0.6 0.6 0.6], 'LineWidth',0.5, ...
        'LineStyle','-.', 'Marker', mkr(mod(i,14)+2));
end

%% �г��������չʾ����
   
    figure('name','Delays');
    xlabel('Location x(t)'); ylabel('Delay (s)');
    title('Delays');
    subplot(M,1,1);
for i=1:M
    subplot(M,1,i);
    title(['Delays (train = ' num2str(i) '), solid Departure, dash Arrival']);
    hold on;
    time=createTime(ArriveNum(i));
    % actual arrival 
    arrdepAct=TimeSchedule.TimeArrive(i,1:length(time));
    % planned arrival
    arrdepPlan=TimeSchedule.TimePlan(i,1:length(time));
    delayDep=arrdepAct(1:2:end-1)-arrdepPlan(1:2:end-1);
    delayArr=arrdepAct(2:2:end)-arrdepPlan(2:2:end);
    plot(time(1:2:end-1), delayDep,'-',  'Color', cm(i,:), 'Marker', mkr(mod(i,14)+1));
    plot(time(2:2:end), delayArr,'--','Color', cm(i,:),'Marker', mkr(mod(i,14)+1),...
        'LineWidth', 1.25);
end



