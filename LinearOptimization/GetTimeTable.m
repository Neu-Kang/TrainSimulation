function [firstTimetable,firstMPRT,firstPM,Ts,Tz,conInformM,delayM]=GetTimeTable(timetable,delay)

% ����1�����timetable�ṹ���
% ����2�������������ʹ��վ̨�����޸�
% ����3��
%{ 

ת��������
������ת��Ϊ����


%}
%168 x 1
timetable=timetable';
[row_num,train_num] = size(timetable);
station_num = (row_num+1)/2;
firstTimetable=zeros(row_num,train_num);
firstTimetable=timetable;
 
firstMPRT=[14 7 8 10 10 8 7 14];    % ������С������ʱ��
departure = firstTimetable(2:2:end,:);
arrive = firstTimetable(1:2:end-1,:);
firstPM = departure-arrive;
firstPM(firstPM>0)=1;  % ͣ������ 0����ͣ�� 1��ͣ��
during = firstTimetable(3:2:end,:)-departure;

Ts=1;                      % ��Сͣ��վʱ��
Tz=4;                      % ��С׷�ټ��2
Tqs=2;                     % �г����𳵸���ʱ��
Tts=1;                     % ͣ������ʱ��


delayM=[
    130   % ���ʱ��
    1   % ����г�
    1   % ��㳵վ
    1   % �������α�ʶ
    1   %������ͣ�1��վ��㣻2վ�����
];
delayM=delay; %����ʱ��
% ������Ϣ����
% ������һ���������Σ�0��������ʼ���г���1��������һ���������Σ�2�������ڶ�����������
% ������һ���������ζ�Ӧ��һ�����ε��г��ţ�0��������ʼ���г�
conInformM=[
    2 2 0 1 0 1 1 2 1 1 2
    1 2 0 3 0 4 5 6 6 7 7
];

end