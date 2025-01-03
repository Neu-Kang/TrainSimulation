function u_real=LMIsolve(M,N,x_real,control)
%�����ܣ���LMIԼ������С������Ŀ��
% Q=0.1I18 R=0.1I14  
% 1 �õ���� 2 �õ�����  3 ��ʵ��

setlmis([]);

%��֪����
d=control.ckd; %�˿͵������ʱ仯��Χ
za=control.d; %�˿͵���������ͣվʱ���ϵ
lamda=control.ckpoint; %�˿͵����������ĵ�

d=0; %�˿͵������ʱ仯��Χ
za=0.2; %�˿͵���������ͣվʱ���ϵ
lamda=0.4; %�˿͵����������ĵ�

c1=(1/2)*((-za*(lamda+d)/(1-za*(lamda+d)))+(-za*(lamda-d)/(1-za*(lamda-d))));
d1=(1/2)*((-za*(lamda-d)/(1-za*(lamda-d)))-(-za*(lamda+d)/(1-za*(lamda+d))));
c2=(1/2)*((1/(1-za*(lamda-d)))+(1/(1-za*(lamda+d))));
d2=(1/2)*((1/(1-za*(lamda+d)))-(1/(1-za*(lamda-d))));
q=1; %Ŀ�꺯��������Ȩ��
r=1; %Ŀ�꺯���п�������Ȩ��

%LMI����֪����
%A M x M
A11=diag(ones(1,M-N-1),1);
A12=zeros(M-N,N);
A12(M-N,1)=1;
A21=zeros(N,M-N);
A21(N,1)=c2;
A22=diag(c1*ones(1,N))+diag(c2*ones(1,N-1),1);
A=[A11 A12;A21 A22];
%B M x N
B1=zeros(M-N,N);
B2=diag(c1.*ones(1,N));
B=[B1;B2];
%C1 M x M
C11=zeros(M-N,M);
C121=zeros(N,M-N);
C122=diag(d1*ones(1,N));
C1=[C11;C121 C122];
%C2 M x M
C21=zeros(M-N,M);
C221=zeros(N,M-N);
C222=diag(d2*ones(1,N-1),1);
C2=[C21; C221 C222];
%C3 N x N
C3=diag(d1.*ones(1,N));
%L M x N
L1=zeros(M-N,N);
L2=diag(1.*ones(1,N));
L=[L1;L2];
%��������
Q=diag(q.*ones(1,M));
Q(1:N,1:N)=diag(0.01*q.*ones(1,N));
R=diag(r.*ones(1,N));
I1=diag(ones(1,M));
IM=diag(ones(1,M));
IN=diag(ones(1,N));
P=diag(ones(1,M));

%���������ϵͳ���
X=x_real; 
U1=20; %����
U2=20; %����
Ul=(U1/2+U2/2)-abs((U1/2-U2/2)); %Ul��ֵ


%����������
a=lmivar(2,[1 1]);
Y=lmivar(2,[N M]);
Z=lmivar(1,[M 1]); % ����һ���Գƾ������ Z
H=lmivar(1,[N 1]);
theta1=lmivar(2,[1,1]);
theta2=lmivar(2,[1,1]);
theta3=lmivar(2,[1,1]);

%% ����1 

% %��һ��
lmiterm([1 1 1 Z],-1,1);
lmiterm([1 1 2 Z],1,A'); 
lmiterm([1 1 2 -Y],1,B');
lmiterm([1 1 3 Z],1,C1');
lmiterm([1 1 4 Z],1,C2');
lmiterm([1 1 5 -Y],1,C3');
lmiterm([1 1 6 Z],1,1);
lmiterm([1 1 7 -Y],1,1);
%�ڶ��� ��0������������
lmiterm([1 2 2 Z],-1,1); lmiterm([1 2 2 theta1],1,I1*I1'); 
lmiterm([1 2 2 theta2],1,I1*I1'); lmiterm([1 2 2 theta3],1,L*L');
%������
lmiterm([1 3 3 theta1],-1,IM);
%������
lmiterm([1 4 4 theta2],-1,IM);
%������
lmiterm([1 5 5 theta3],-1,IN);
%������
lmiterm([1 6 6 a],-1,inv(Q));
%������
lmiterm([1 7 7 a],-1,inv(R));


%% ����2
lmiterm([-2 1 1 0],1);
lmiterm([-2 2 1 0],X);
lmiterm([-2 2 2 Z],1,1);

% ����3
lmiterm([-3 1 1 H],1,1);
lmiterm([-3 1 2 Y],1,1);
lmiterm([-3 2 2 Z],1,1);

%% ��������0Լ��
lmiterm([-4 1 1 a],1,1);

lmiterm([-5 1 1 theta1],1,1);

lmiterm([-6 1 1 theta2],1,1);

lmiterm([-7 1 1 theta3],1,1);

lmiterm([-8 1 1 Z],1,1);

lmiterm([-9 1 1 H],1,1);
%% ������Լ��
for i = 1:N
    % ������ʱ���������ɶԽ�Ԫ��
    tempZL = zeros(1, N);
    tempZL(i) = 1;
    tempZR = zeros(N, 1);
    tempZR(i) = 1;
    lmiterm([(9+i) 1 1 H], tempZL, tempZR); % ���� Z(i, i) �� LMI ��
    lmiterm([-(9+i) 1 1 0], Ul^2);  % ���� -ul �� LMI �У��γ� Z(i,i) - ul < 0
end

% for i = 1:N
%    for j = 1:M
%        tempZL = zeros(1, N);
%        tempZL(i) = 1;
%        tempZR = zeros(M, 1);
%        tempZR(j) = 1;
%        lmiterm([9+N+(i-1)*M+j 1 1 Y],tempZL,tempZR);       
%    end    
% end

%% LMI����
lmisys=getlmis; %��ȡLMI

% ��С��Ŀ�꺯��
n = decnbr(lmisys);  % ϵͳ���߱�������
c = zeros(n, 1);  % ȷ������c��ά��

% ����Ŀ�꺯��c��ʹ����С������a
for j = 1:n
    if ismember(j, decinfo(lmisys, a))  % ������a�ľ��߱�������
        c(j) = 1;  % ���ö�Ӧa�ľ��߱�������Ϊ1������Ϊ0
    end
end

% ��С��Ŀ�꺯��
[copt, xopt] = mincx(lmisys, c);
a=dec2mat(lmisys,xopt,1);
Y=dec2mat(lmisys,xopt,2);
Z=dec2mat(lmisys,xopt,3);
K=Y*inv(Z);
U=K*X;
xl=X(9:end);
u_real=roundn(U,-1);


end 