function minEntropy=Get_fitness(data,alpha_0,K_0,varargin)
% ��Ӧ�Ⱥ���,��С����VMD�����ľֲ�������
% ����VMD��Ʒ������������
nvarargin = length(varargin);
alpha = alpha_0; % �ʶȵĴ���Լ��/�ͷ�����
tau = 0; % �������ޣ�û���ϸ�ı����ִ�У�
K = K_0; % �ֽ��ģ̬����
DC = 1; % ��ֱ������
init = 1; % omegas�ľ��ȳ�ʼ��
tol = 1e-7;  %����׼�򣬿����ʵ����� 

[u,~,~]=VMD(data, alpha, tau, K, DC, init, tol); %����VMD

minEntropy=Get_Entropy(u,K,nvarargin); %���������

  

end