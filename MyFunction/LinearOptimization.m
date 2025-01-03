function [timetableAfter,OP]=LinearOptimization(Tnum,Pnum,delay,timetable,varargin)
%{
�޸Ĳ��֣� 
1.������
2.������
%}


%% ������ʼ��
if length(varargin)==1
    if varargin{1}== 1
        timetableAfter=timetable;
        OP=1;
       return 
    end
end
OP=1;
[firstTimetable,firstMPRT,firstPM,Ts,TsJ,Tz,conInformM,delayM]=GetTimeTable(Tnum,Pnum,timetable,delay);
Ar_Station_Delay = delayM(:,delayM(5,:)==1);
Ar_Station_Delay_Train = Ar_Station_Delay(2,:);
Ar_Station_Delay_Time = Ar_Station_Delay(1,:);
Ar_Station_Delay_Station = Ar_Station_Delay(3,:);
Ar_Station_Delay_TraNum = size(Ar_Station_Delay,2);
ActArSta_Time = firstTimetable(Ar_Station_Delay_Station*2-1,:);
for i=1:size(delayM,2)
    ActArSta_Time(1,Ar_Station_Delay_Train(i)) = firstTimetable(1,Ar_Station_Delay_Train(i)) + Ar_Station_Delay_Time(i);
end
ActArFirstSta_Time = ActArSta_Time;
timetable(Ar_Station_Delay_Train,1)=timetable(Ar_Station_Delay_Train,1)+Ar_Station_Delay_Time';
[m,train_num] = size(firstTimetable);
station_num = m/2;
M = 2000000;
sec_num = station_num-1;
T_max = Tz;
T_min = 1;
H=5;%��Ȼ�����Ҫ�趨

% �������߱���
tic
depart_time = sdpvar(sec_num,train_num); % ����ʱ�� xij��ʾ��iվ��j���г��ķ���ʱ��
arrive_time = sdpvar(sec_num,train_num);  % ��վʱ�� yij��ʾ��i+1վ��j���г��ĵ�վʱ��
isStop = binvar(sec_num,train_num,'full'); %����*�г� �жϸ�վ�Ƿ�վ���
depart_order = binvar(train_num,train_num,sec_num,'full');% ��ʾiվ�г��ķ���˳�������depart_order��i,j,k��=1ʱ����վk���г�i����j����
adorder = binvar(train_num,train_num,sec_num,'full');% ���߱���sita ��ʾͬһ��վ�����г���ĵ���˳��adorder��i,j,k��i�ӳ�վ��k+1����������j���ﳵվ��k+1��
isDelay = binvar(sec_num,train_num,'full'); %�жϸ�վ�Ƿ�վ��㣺isDelay��j,i��=1,�г�i����j+1վ���
%track_nums = [2,3,4,2,5, 2, 2, 3, 2, 3, 5];  % �˴���Ҫ���ݳ�����վ�����޸ģ����Գ�����7վ��������7��Ԫ��
%track_nums = [1,1,1,1,1, 1, 1, 1, 1, 1, 1];
%track_nums = [10,10,10,10,10, 10, 10, 10, 10, 10, 10];
track_nums = ones(sec_num+1);
max_track_num = max(track_nums);%������·�����վ�ĵ����ߣ����ߣ�����
T_su = 4;
T_sd = 2;
z=binvar(station_num,max_track_num,train_num,'full'); %��վ*�г�*�ɵ����� ÿ���г���ÿ����վʹ�õĹɵ�
overrunbin = binvar(sec_num,train_num*(train_num-1)/2,'full');
% overrunbin(�г�Խ�й�ϵ01����):����1��1�ڳ�2ǰ
% ����1:Trian1*2 Train1*3 Train1*4��(��1��������Խ��Լ��)Train2*3 Train2*4������2���1�������������
% ����2: Trian1*2 Train1*3 Train1*4��(��1��������Խ��Լ��)Train2*3 Train2*4������2���1�������������
% ��.
C = [];


for i=1:train_num
    % ����ʱ��Լ��:a1i>=a1i*+delaytime
    C = [C,depart_time(1,i)>=(ActArFirstSta_Time(1,i)+firstPM(Ar_Station_Delay_Station(1),i)*Ts)];

    for j=1:sec_num
        %�ƻ�����ʱ��Լ��
        C = [C,depart_time(j,i)>=firstTimetable(2*j,i)];
        %��Сͣվʱ��Լ����ȱ�ٵ�һվ��
        if j>1
            C = [C,depart_time(j,i)-arrive_time(j-1,i)>=firstPM(j,i)*Ts];
        end
        switch delay
            case 1
                if j==4
                    C = [C,depart_time(j,i)-arrive_time(j-1,i)>=firstPM(j,i)*TsJ];
                end
            case 2
                if j==4
                    C = [C,depart_time(j,i)-arrive_time(j-1,i)>=firstPM(j,i)*TsJ];
                end
            case 3
                if j==4||j==8
                    C = [C,depart_time(j,i)-arrive_time(j-1,i)>=firstPM(j,i)*TsJ];
                end
       
        end
            
            
            
        %��������ʱ��Լ��:dij-aij>=qi
        C = [C,arrive_time(j,i)-depart_time(j,i)>=firstMPRT(1,j)];
        %����ʱ�����ȡֵԼ��
        C = [C,arrive_time(j,i)<=1440];
        C = [C,depart_time(j,i)<=1440];
        %�Ƿ����Լ��
         C = [C,arrive_time(j,i)-isDelay(j,i)+M*(1-isDelay(j,i))>=firstTimetable(2*(j+1)-1,i)];
         C = [C,arrive_time(j,i)+M*(1-isDelay(j,i))<=firstTimetable(2*(j+1)-1,i)+M];
    end
end

% �г�ͣվʱ��Ҫ�������г��������
% i = 1;
% for j=1:sec_num
%     if j>7
% %         C = [C,depart_time(j,i)-arrive_time(j-1,i)==firstTimetable(2*j,i)-firstTimetable(2*j-1,i)];
%     else
%         if j>1
%             C = [C arrive_time(j,i)==firstTimetable(2*j-1,i)];
%         else
%             C = [C depart_time(j,i)==firstTimetable(2*j,i)];
%         end
%         
%     end
%     
% end
% 
% for i=2:train_num
%     for j=1:sec_num
%         if j>1
%             C = [C,depart_time(j,i)-arrive_time(j-1,i)==firstTimetable(2*j,i)-firstTimetable(2*j-1,i)+(arrive_time(j-1,i)-arrive_time(j-1,i-1))*0.05*0.5];
%         end
%     end
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%����ʱ������������ͣ����%%%%%%%%%%%%%%%%%%%
for i=1:(sec_num-1)
    for j=1:train_num
        for k=1:train_num
            if j~=k
                C = [C,depart_time(i+1,k)-arrive_time(i,j)+M*adorder(k,j,i+1)>=Tz];
                C = [C,depart_time(i+1,k)-arrive_time(i,j)+M*adorder(k,j,i+1)>=1];
                C = [C,depart_time(i+1,k)-arrive_time(i,j)-M*(1-adorder(k,j,i+1))<=-Tz];
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%����ʱ����������ͣ����%%%%%%%%%%%%%%%%%%%
% for i=1:(sec_num-1)
%     for j=1:train_num
%         for k=1:train_num
%             if j~=k
%                 C = [C,depart_time(i+1,k)-arrive_time(i,j)+M*(2-departorder(k,j,i)-isStop(i+1,k))>=Tz];
%             end
%         end
%     end
% end
% 
% %ͣվʱ��Լ��
% for i=1:train_num
%     for j=1:(sec_num-1)
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)>=Ts*isStop(j+1,i)]; 
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)<=M*isStop(j+1,i)]; 
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%ͣվ�ƻ�Լ��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:train_num
%     for j=1:sec_num
% 
%         C = [C,isStop(j,i)==firstPM(j,i)];
% 
%     end
% end
% 
% %ͣվʱ��Լ��
% for i=1:train_num
%     for j=1:(sec_num-1)
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)>=Ts*isStop(j+1,i)]; 
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)<=M*isStop(j+1,i)]; 
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%��ͣ����ʱ��Լ��%%%%%%%%%%%%%%%%%%%%%%
% for i=1:train_num
%     for j=2:(sec_num-1)
% 
%         %��������ʱ��Լ��:dij-aij>=qi
%         C = [C,arrive_time(j,i)-depart_time(j,i)>=firstMPRT(1,j)+isStop(j,i)*T_su+isStop(j+1,i)*T_sd];
% 
%     end
% end
% %ͣվʱ��Լ��
% for i=1:train_num
%     for j=1:(sec_num-1)
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)>=Ts*isStop(j+1,i)]; 
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)<=M*isStop(j+1,i)]; 
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%��������ʱ����Լ�������(�����������г��Ƿ�ͣ��)%%%%%%%%%%%%%%%%%
% for i = 1:sec_num
%     lefttrain_num = train_num-1;
%     accumulatetrain_num = 0;
%     for j = 1:train_num
%         for k=1:lefttrain_num
%             C = [C,depart_time(i,j)-depart_time(i,j+k)>=Tz-(1-overrunbin(i,accumulatetrain_num+k))*M];
%             C = [C,depart_time(i,j+k)-depart_time(i,j)>=Tz-overrunbin(i,accumulatetrain_num+k)*M];
%             C = [C,arrive_time(i,j)-arrive_time(i,j+k)>=Tz-(1-overrunbin(i,accumulatetrain_num+k))*M];
%             C = [C,arrive_time(i,j+k)-arrive_time(i,j)>=Tz-overrunbin(i,accumulatetrain_num+k)*M];    
%         end
%         accumulatetrain_num = accumulatetrain_num+lefttrain_num;
%         lefttrain_num = lefttrain_num-1;
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%��������ʱ����Լ�������(�����������г��Ƿ�ͣ��)%%%%%%%%%%%%%%%%
% for i = 1:sec_num
%     for j = 1:train_num
%         for k = (j+1):train_num            
%             C = [C,depart_time(i,j)-depart_time(i,k)>=Tz-(1-depart_order(k,j,i))*M];
%             C = [C,depart_time(i,k)-depart_time(i,j)>=Tz-depart_order(k,j,i)*M];
%             C = [C,arrive_time(i,j)-arrive_time(i,k)>=Tz-(1-depart_order(k,j,i))*M];
%             C = [C,arrive_time(i,k)-arrive_time(i,j)>=Tz-depart_order(k,j,i)*M];
%         end 
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%��������ʱ����Լ��ȫ����(�����������г��Ƿ�ͣ��)%%%%%%%%%%%%%%%%%
for i = 1:sec_num
    for j = 1:train_num
        for k = 1:train_num
            if j~=k
                C = [C,depart_order(j,k,i)+depart_order(k,j,i)==1];
                C = [C,depart_time(i,j)-depart_time(i,k)>=Tz-(1-depart_order(k,j,i))*M];
                C = [C,arrive_time(i,j)-arrive_time(i,k)>=Tz-(1-depart_order(k,j,i))*M];
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%��������ʱ����Լ��ȫ����(���������г��Ƿ�ͣ������ʱ����Լ��)%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i = 1:(sec_num-1)
%     for j = 1:train_num
%         for k = 1:train_num
%             if j~=k
%                 C = [C,depart_order(j,k,i)+depart_order(k,j,i)==1];
%                 C = [C,depart_time(i+1,k)-depart_time(i+1,j)>=T_max-(2-depart_order(j,k,i+1)-isStop(i+1,j))*M];
%                 C = [C,abs(depart_time(i+1,k)-depart_time(i+1,j))>=T_min-(1-abs(depart_order(j,k,i+1)-isStop(i+1,j)))*M];
%                 C = [C,arrive_time(i,k)-arrive_time(i,j)>=T_max-(2-depart_order(j,k,i)-isStop(i+1,j))*M];
%                 C = [C,abs(arrive_time(i,k)-arrive_time(i,j))>=T_min-(1-abs(depart_order(j,k,i)-isStop(i+1,j)))*M];
%             end
%         end
%     end
% end
% 
% 
% %��������ʱ����Լ��ȫ����(���������г��Ƿ�ͣ����վ����ʱ����Լ��)
% for i = 1:1
%     for j = 1:train_num
%         for k = 1:train_num
%             if j~=k
%                 C = [C,depart_time(i,j)-depart_time(i,k)>=Tz-(1-depart_order(k,j,i))*M];
%             end
%         end
%     end
% end
% %��������ʱ����Լ��ȫ����(���������г��Ƿ�ͣ��ĩվ����ʱ����Լ��)
% for i = sec_num:sec_num
%     for j = 1:train_num
%         for k = 1:train_num
%             if j~=k
%                 C = [C,depart_order(j,k,i)+depart_order(k,j,i)==1];
%                 C = [C,arrive_time(i,j)-arrive_time(i,k)>=Tz-(1-depart_order(k,j,i))*M];
%             end
%         end
%     end
% end
% 
% %Խ��Լ��
% for i = 1:sec_num
%     for j = 1:train_num
%         for k = 1:train_num
%             if j~=k
%                 C = [C,depart_time(i,j)-depart_time(i,k)>0-(1-depart_order(k,j,i))*M];
%                 C = [C,arrive_time(i,j)-arrive_time(i,k)>0-(1-depart_order(k,j,i))*M];
%             end
%         end
%     end
% end
% 
% %�Ƿ�ͣվԼ��
% for i=1:train_num
%     for j=1:(sec_num-1)
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)>=Ts*isStop(j+1,i)]; 
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)<=M*isStop(j+1,i)]; 
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��վ����Լ��(�����ɵ�����)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:(sec_num-1)
%     for j=1:train_num
%         %��վ����Լ��
% %         x=0;
% %         y=0;
%         C = [C,sum(depart_order(:,j,i))-depart_order(j,j,i)-(sum(adorder(:,j,i+1))-adorder(j,j,i+1))+isStop(i+1,j)<=track_nums(i+1)];
%         %C = [C,sum(depart_order(:,j,i))-depart_order(j,j,i)-(sum(adorder(:,j,i+1))-adorder(j,j,i+1))+1<=track_nums(i+1)];
%         for k=1:train_num
%             if j~=k
%                 %C = [C,adorder(j,k,i+1)+adorder(k,j,i+1)==1];%��Ӻ��г�˳�򲻱�
%                 C = [C,arrive_time(i,j)-depart_time(i+1,k)+M*(1-adorder(k,j,i+1))>=Tz];
%                 C = [C,depart_time(i+1,k)-arrive_time(i,j)+M*adorder(k,j,i+1)>0];
% %                 x=x+depart_order(k,j,i);
% %                 y=y+adorder(k,j,i+1);
%             end
%         end
% %        C = [C,x-y+isStop(i+1,j)<=track_nums(i+1)];
%     end
% end
% 
% %ͣվʱ��Լ��
% for i=1:train_num
%     for j=1:(sec_num-1)
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)>=Ts*isStop(j+1,i)]; 
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)<=M*isStop(j+1,i)]; 
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��վ����Լ�������ɵ�����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:(sec_num-1)
    itrack_num = track_nums(i+1);
    for j=1:train_num
        C = [C,sum(sum(z(i+1,1:itrack_num,j)))==1];
        for k=1:train_num
            if j~=k
                for ii=1: itrack_num
                C = [C,arrive_time(i,j)-depart_time(i+1,k)+M*(3-depart_order(k,j,i)-z(i+1,ii,j)-z(i+1,ii,k))>=Tz];    
                end
            end
        end
    end
end
% % 
% %ͣվʱ��������ͨ��Լ��
% for i=1:train_num
%     for j=1:(sec_num-1)
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)>=Ts*isStop(j+1,i)]; 
%         C = [C,depart_time(j+1,i)-arrive_time(j,i)<=M*isStop(j+1,i)]; 
%         C = [C,1-isStop(j+1,i)<=z(j+1,1,i)]; 
%         C = [C,z(j+1,1,i)<=1-isStop(j+1,i)]; 
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% ����
ops = sdpsettings('verbose',0,'solver','cplex');
%% Ŀ�꺯���������ʱ����С
% DelayFa=0;
% for j=9:size(depart_time,1)
%     DelayFa=DelayFa+abs(depart_time(j,1)-depart_time(j-8,15)-H);
% end
% for i=2:size(depart_time,2)%�г���
%    for  j=1:size(depart_time,1)%վ̨��
%        DelayFa=DelayFa+abs(depart_time(j,i)-depart_time(j,i-1)-H);
%    end
% end
% for i=1:size(depart_time,2)%�г���
%    for  j=1:size(depart_time,1)-1%վ̨��
%        DelayFa=DelayFa+abs((depart_time(j+1,i)-depart_time(j,i))-(firstTimetable(2*(j+1),i)-firstTimetable(2*j,i)));
%    end
% end

DelayFa = sum(sum(depart_time-firstTimetable(4:2:end,:))); % ����ʱ��
DelayDao = sum(sum((arrive_time-firstTimetable(3:2:end,:)).*isDelay))+sum(sum((arrive_time-firstTimetable(3:2:end,:)).*(0.3*(isDelay-1)))); % ��վʱ��
J = DelayFa+DelayDao;
saveampl(C,J,'mymodel');
result = solvesdp(C,J,ops);%Ĭ�����Ŀ�꺯����Сֵ
if result.problem ~= 0 % problem =0 �������ɹ�
    disp('������');
end
%ͨ��value������ȡ���߱������Ž�ȡֵ
depart_time=value(depart_time);
arrive_time=value(arrive_time);
adorder = value(adorder);
departorder = value(depart_order);
isDelay=value(isDelay);
isStop=value(isStop);
track = value(z);
toc
%�����������ʱ�̱�
train_list=zeros(sec_num*2,train_num);

train_list(1:2:end,:)=depart_time;
train_list(2:2:end,:)=arrive_time;
train_list = round(train_list);

timetableAfter=[timetable(:,1) train_list' timetable(:,end)];

end
