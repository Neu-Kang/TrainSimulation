function Uik=Get_iterative(Tnum,Pnum,TimeSchedule,Tr,Plat,control)
%{
函数功能：根据上一批次的同一时刻的控制率、输出量以及状态偏差量计算的得到当前时刻的控制率


%}

%已知变量
M=Tr.M; %16辆列车
N=Plat.N; %8个站台
p=2; %一个批次数量\预测范围
m=2; %算法的控制范围，控制范围越小计算压力越小

%权重值（自己设置）
% c=control.ck(Pnum); %客流与停车时间关系
c=control.ckpoint; %客流与停车时间关系
c1=-c/(1-c);
c2=1/(1-c);
q=3; %误差大小权重
ukk=[]
r=1 ; %控制率大小权重

%与仿真器进行对接变量
%Xk 当前时刻的状态量

if Tr.TrCir(Tnum)==0
    Xk=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Xk=[Xk ;TimeSchedule.TimeError(trnum,Pnum+Plat.N*Tr.TrCir(Tnum)+1)];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Xk=[Xk ;TimeSchedule.TimeError(trnum,plnum+Plat.N*Tr.TrCir(Tnum))];
        j=j+1;
    end   
    Xk1=zeros(M,1); %目前时刻上一批次的状态量
    uik1=zeros(N,1); %保存所有控制率
    yP_k1=zeros(p*M,1); %横向p维度,纵向M维度
else
    Xk=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Xk=[Xk ;TimeSchedule.TimeError(trnum,Pnum+Plat.N*Tr.TrCir(Tnum))];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Xk=[Xk ;TimeSchedule.TimeError(trnum,plnum+Plat.N*Tr.TrCir(Tnum))];
        j=j+1;
    end
    %上一批次的当前时刻状态量
    Xk1=[];
    for i=M:-1:N+1
        trnum=mod2(Tnum-i+N-1,M);
        Xk1=[Xk1 ;TimeSchedule.TimeError(trnum,Pnum+Plat.N*(Tr.TrCir(Tnum)-1))];
    end
    j=0;
    for i=N:-1:1
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        Xk1=[Xk1 ;TimeSchedule.TimeError(trnum,plnum+Plat.N*(Tr.TrCir(Tnum)-1))];
        j=j+1;
    end
    %上一批次的控制率
    uik1=[];
    j=1;
    for i=N-1:-1:0
        trnum=mod2(Tnum-i+N-1,M);
        plnum=mod2(Pnum-j+1,N);
        uik1=[uik1 ;TimeSchedule.TimeUk(trnum,plnum+Plat.N*(Tr.TrCir(Tnum)-1))];
        j=j+1;
    end
    %预测范围p内所有状态向量
    yP_k1=[];
    for z=1:p
       %上一批次的当前时刻状态量
        Tnump=mod2(Tnum+z,M);
        for i=M:-1:N+1
            trnum=mod2(Tnump-i,M);
            yP_k1=[yP_k1 ;TimeSchedule.TimeError(trnum,Pnum+Plat.N*(Tr.TrCir(Tnum)-1))];
        end
        j=0;
        for i=N:-1:1
            trnum=mod2(Tnump-i,M);
            plnum=mod2(Pnum-j,N);
            yP_k1=[yP_k1 ;TimeSchedule.TimeError(trnum,plnum+Plat.N*(Tr.TrCir(Tnum)-1))];
            j=j+1;
        end

    end

end
uik=zeros(N,1);

%LMI已知变量
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
B=[B1;B2]; %M X N 矩阵  
%G 
G=zeros(p*size(B,1),m*size(B,2));
for i=1:m
    for j=p-i+1:-1:1
       G((i-1+j-1)*size(B,1)+1:(i-1+j)*size(B,1),(i-1)*size(B,2)+1:i*size(B,2))=A^(j-1)*B; 
    end
end
F=A;
for i=2:p 
    F=[F ; A^i];
end
Q1=[zeros(1, M-N), repmat(q, 1, N)];
Q=diag(repmat(Q1,1,p));
R=r*eye(m*N);
H=inv(G'*Q*G+R)*G'*Q;
T=[eye(N) zeros(N,size(H,1)-N)];
H1=T*H;
AXk=Xk-Xk1; 
%求解控制率
uik=uik1+H1*(yP_k1+F*AXk);
uikfir=uik1;
uiksec=H1*(yP_k1+F*AXk);
Uik=uik(1);
z=H1*F;
y=F*AXk;

% Uik=-10*5*abs(uik(1));
% error=TimeSchedule.TimeError(Tnum,Pnum+Plat.N*Tr.TrCir(Tnum));
% Uik=-Plat.sequence(Tr.TrCir(Tnum)+1)*error; 


end
