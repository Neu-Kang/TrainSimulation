clear all;
close all;
warning off;

%��ȡ����������Ӧģ��
a = readfis('fuzzpid'); 

%�������ݺ���ģ��
sys = tf(133,[1,25,0]);

%������ʱ��ģ��ת��Ϊ��ɢʱ��ģ��
ts = 0.001;     %��ɢ�Ĳ���ʱ��
dsys = c2d(sys,ts,'z'); %z�任������ɢ��Ϊ����
[num,den] = tfdata(dsys,'v'); %���ʴ��ݺ�������
%numΪ����ϵ���������� denΪ��ĸϵ���������� ��0 1��Ϊ 1Ϊ������ 

%����������������������������仯�ٶ�
u_1 = 0;
u_2 = 0;
y_1 = 0;
y_2 = 0;
e_1 = 0;
ec_1 = 0;

%�������ϵ��
kp0 = 0;    

for k = 1:1:1000
    time(k) = k *ts;  %��¼ʱ��
    r(k) = 1;  %Ŀ�����
    k_pid = evalfis([e_1,ec_1],a);%ģ����e_1,ec_1�õ�?kp,?ki 
    kp(k) = kp0 + k_pid(1);%����kp,����kp
    u(k) = kp(k)*e_1;%����������
    y(k) = -den(2)*y_1-den(3)*y_2+ num(2)*u_1 + num(3)* u_2;%��ɢϵͳ���ݺ��������ʱ��
    e(k) = r(k) - y(k);%��ȡ���
    u_2 = u_1;
    u_1 = u(k);
    y_2 = y_1;
    y_1 = y(k);
    ec(k) = e(k) - e_1;
    e_1 = e(k);
    ec_1 = ec(k);
end
%��ʼ�����̶�kpֵ
u_1 = 0;
u_2 = 0;
y_1 = 0;
y_2 = 0;
e_1 = 0;
ec_1 = 0;
kp0=kp(k);
kp2=kp0*ones(1,k);
for k = 1:1:1000
    time2(k) = k *ts;  %��¼ʱ��
    r(k) = 1;  %Ŀ�����
    u2(k) = kp0*e_1;%����������
    y2(k) = -den(2)*y_1-den(3)*y_2+ num(2)*u_1 + num(3)* u_2;%��ɢϵͳ���ݺ��������ʱ��
    e(k) = r(k) - y(k);%��ȡ���
    u_2 = u_1;
    u_1 = u(k);
    y_2 = y_1;
    y_1 = y(k);
    ec(k) = e(k) - e_1;
    e_1 = e(k);
    ec_1 = ec(k);
end

%ģ������Ӧpidͼ��
figure(1);
subplot(3,1,1);
plot(time,r,'r',time,y,'b:','linewidth',2);
xlabel('time(s)');
ylabel('r,y');
legend('Ideal position','Practical position');
subplot(3,1,2);
plot(time,kp,'r','linewidth',2);
xlabel('time(s)');
ylabel('kp');
subplot(3,1,3);
plot(time,u,'r','linewidth',2);
xlabel('time(s)');
ylabel('Control input');

%ֱ�Ӳ�������Ӧ���ͼ��
figure(2);
subplot(3,1,1);
plot(time2,r,'r',time,y2,'b:','linewidth',2);
xlabel('time(s)');
ylabel('r,y');
legend('Ideal position','Practical position');
subplot(3,1,2);
plot(time2,kp2,'r','linewidth',2);
xlabel('time(s)');
ylabel('kp');
subplot(3,1,3);
plot(time2,u2,'r','linewidth',2);
xlabel('time(s)');
ylabel('Control input');
