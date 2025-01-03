function minEntropy=Get_Entropy(u,K,nvarargin)
for i = 1:K
	uh= abs(hilbert(u(i,:))); %��С�����ؼ��㹫ʽ��
	uhs = uh/sum(uh);
    ssum=0;
	for ii = 1:size(uhs,2)
		bb = uhs(1,ii)*log(uhs(1,ii));
        ssum=ssum+bb;
    end
    Entropy(i,:) = -ssum;%ÿ��IMF�����İ�����
%     disp(['IMF',num2str(i),'�İ�����Ϊ��',num2str(Entropy(i,:))])
end
if nvarargin == 1
   minEntropy = Entropy;%��ȡ���а�����
else
    minEntropy = min(Entropy);%��ȡ�ֲ���С�����أ�һ���������Ż��㷨�Ż�VMD����Ϊ��С��Ӧ�Ⱥ���ʹ��
end


% disp(['�ֲ���С������Ϊ��',num2str(minEntropy)])
end
