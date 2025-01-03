function u_real=LMIsolve(M,N,x_real,control)
%程序功能：在LMI约束下最小化线性目标
% Q=0.1I18 R=0.1I14  
% 1 得到结果 2 得到晚点  3 做实验

setlmis([]);

%已知变量
d=control.ckd; %乘客到达速率变化范围
za=control.d; %乘客到达速率与停站时间关系
lamda=control.ckpoint; %乘客到达速率中心点

d=0; %乘客到达速率变化范围
za=0.2; %乘客到达速率与停站时间关系
lamda=0.4; %乘客到达速率中心点

c1=(1/2)*((-za*(lamda+d)/(1-za*(lamda+d)))+(-za*(lamda-d)/(1-za*(lamda-d))));
d1=(1/2)*((-za*(lamda-d)/(1-za*(lamda-d)))-(-za*(lamda+d)/(1-za*(lamda+d))));
c2=(1/2)*((1/(1-za*(lamda-d)))+(1/(1-za*(lamda+d))));
d2=(1/2)*((1/(1-za*(lamda+d)))-(1/(1-za*(lamda-d))));
q=1; %目标函数中误差的权重
r=1; %目标函数中控制量的权重

%LMI中已知变量
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
%常数定义
Q=diag(q.*ones(1,M));
Q(1:N,1:N)=diag(0.01*q.*ones(1,N));
R=diag(r.*ones(1,N));
I1=diag(ones(1,M));
IM=diag(ones(1,M));
IN=diag(ones(1,N));
P=diag(ones(1,M));

%后期与仿真系统结合
X=x_real; 
U1=20; %上限
U2=20; %下限
Ul=(U1/2+U2/2)-abs((U1/2-U2/2)); %Ul数值


%求解变量定义
a=lmivar(2,[1 1]);
Y=lmivar(2,[N M]);
Z=lmivar(1,[M 1]); % 定义一个对称矩阵变量 Z
H=lmivar(1,[N 1]);
theta1=lmivar(2,[1,1]);
theta2=lmivar(2,[1,1]);
theta3=lmivar(2,[1,1]);

%% 矩阵1 

% %第一行
lmiterm([1 1 1 Z],-1,1);
lmiterm([1 1 2 Z],1,A'); 
lmiterm([1 1 2 -Y],1,B');
lmiterm([1 1 3 Z],1,C1');
lmiterm([1 1 4 Z],1,C2');
lmiterm([1 1 5 -Y],1,C3');
lmiterm([1 1 6 Z],1,1);
lmiterm([1 1 7 -Y],1,1);
%第二行 （0不进行描述）
lmiterm([1 2 2 Z],-1,1); lmiterm([1 2 2 theta1],1,I1*I1'); 
lmiterm([1 2 2 theta2],1,I1*I1'); lmiterm([1 2 2 theta3],1,L*L');
%第三行
lmiterm([1 3 3 theta1],-1,IM);
%第四行
lmiterm([1 4 4 theta2],-1,IM);
%第五行
lmiterm([1 5 5 theta3],-1,IN);
%第六行
lmiterm([1 6 6 a],-1,inv(Q));
%第七行
lmiterm([1 7 7 a],-1,inv(R));


%% 矩阵2
lmiterm([-2 1 1 0],1);
lmiterm([-2 2 1 0],X);
lmiterm([-2 2 2 Z],1,1);

% 矩阵3
lmiterm([-3 1 1 H],1,1);
lmiterm([-3 1 2 Y],1,1);
lmiterm([-3 2 2 Z],1,1);

%% 变量大于0约束
lmiterm([-4 1 1 a],1,1);

lmiterm([-5 1 1 theta1],1,1);

lmiterm([-6 1 1 theta2],1,1);

lmiterm([-7 1 1 theta3],1,1);

lmiterm([-8 1 1 Z],1,1);

lmiterm([-9 1 1 H],1,1);
%% 控制量约束
for i = 1:N
    % 定义临时变量以生成对角元素
    tempZL = zeros(1, N);
    tempZL(i) = 1;
    tempZR = zeros(N, 1);
    tempZR(i) = 1;
    lmiterm([(9+i) 1 1 H], tempZL, tempZR); % 添加 Z(i, i) 到 LMI 中
    lmiterm([-(9+i) 1 1 0], Ul^2);  % 添加 -ul 到 LMI 中，形成 Z(i,i) - ul < 0
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

%% LMI计算
lmisys=getlmis; %获取LMI

% 最小化目标函数
n = decnbr(lmisys);  % 系统决策变量个数
c = zeros(n, 1);  % 确定向量c的维数

% 构建目标函数c，使其最小化变量a
for j = 1:n
    if ismember(j, decinfo(lmisys, a))  % 检查变量a的决策变量索引
        c(j) = 1;  % 设置对应a的决策变量索引为1，其余为0
    end
end

% 最小化目标函数
[copt, xopt] = mincx(lmisys, c);
a=dec2mat(lmisys,xopt,1);
Y=dec2mat(lmisys,xopt,2);
Z=dec2mat(lmisys,xopt,3);
K=Y*inv(Z);
U=K*X;
xl=X(9:end);
u_real=roundn(U,-1);


end 