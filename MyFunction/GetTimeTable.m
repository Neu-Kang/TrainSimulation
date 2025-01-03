function [firstTimetable,firstMPRT,firstPM,Ts,TsJ,Tz,conInformM,delayM]=GetTimeTable(Tnum,Pnum,timetable,delay)

% ����1�����timetable�ṹ���
% ����2�������������ʹ��վ̨�����޸�
% ����3��
%{ 

ת��������
������ת��Ϊ����


%}
%168 x 1
if delay==1||delay==3
    fixMPRT=[7 8 12  12 8 7 14 14]; %һȦ������վ̨���������ʱ��
else
    fixMPRT=[8 7 14 14 7 8 12 12]; %һȦ������վ̨���������ʱ��
end
fixN=size(fixMPRT,2); %һȦ�Ĺ̶������
timetable=timetable';
[row_num,train_num] = size(timetable);
station_num = (row_num+1)/2;
firstTimetable=zeros(row_num,train_num);
firstTimetable=timetable;
m=station_num/fixN;
n=mod(station_num,fixN);
firstMPRT=[];    % ������С������ʱ��
for i=1:m
    firstMPRT=[firstMPRT fixMPRT];
end
firstMPRT=[firstMPRT fixMPRT(1:n)];
departure = firstTimetable(2:2:end,:);
arrive = firstTimetable(1:2:end-1,:);
firstPM = departure-arrive;
firstPM(firstPM>0)=1;  % ͣ������ 0����ͣ�� 1��ͣ��
during = firstTimetable(3:2:end,:)-departure(1:1:end-1,:);

Ts=2;                      % ��Сͣ��վʱ��
TsJ=6;                     % ��·վ��Сͣվʱ��
Tz=1;                      % ��С׷�ټ��2
Tqs=0;                     % �г����𳵸���ʱ��
Tts=1;                     % ͣ������ʱ��

if delay==1
    delayM=[
    15 11 8 5 2;   % ���ʱ��
    2 3 4 5 6;   % ����г�
    1 1 1 1 1;   % ��㳵վ
    1 1 1 1 1;   % �������α�ʶ
    1 1 1 1 1;  %������ͣ�1��վ��㣻2վ�����
];
elseif delay==2
    delayM=[
    8 16.3 13.6 10.3 7 4;   % ���ʱ��
    2 3 4 5 6 7;   % ����г�
    1 1 1 1 1 1;   % ��㳵վ
    1 1 1 1 1 1;  % �������α�ʶ
    1 1 1 1 1 1;  %������ͣ�1��վ��㣻2վ�����
];
else
    delayM=[
    50.0, 45.0, 40.0, 35.0, 30.0, 26.0, 21.0, 16.0, 11.0, 6.1,3.0;   % ���ʱ��
    linspace(2,12,11);   % ����г�
    ones(1,11);   % ��㳵վ
    ones(1,11);  % �������α�ʶ
    ones(1,11);  %������ͣ�1��վ��㣻2վ�����
];

end


% ������Ϣ����
% ������һ���������Σ�0��������ʼ���г���1��������һ���������Σ�2�������ڶ�����������
% ������һ���������ζ�Ӧ��һ�����ε��г��ţ�0��������ʼ���г�
conInformM=[
    2 2 0 1 0 1 1 2 1 1 2
    1 2 0 3 0 4 5 6 6 7 7
];

end