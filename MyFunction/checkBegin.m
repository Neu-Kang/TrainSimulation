function y=checkBegin(t,T,Tr)
%��������:����Ƿ���Ӧ�÷����ĳ��������س����кţ����ޣ��򷵻�0��
y=0;
for i=1:Tr.M
    if t>=(i-1)*T&&t<i*T&&Tr.TrB(i)==false
        y=i;
        break;
    end
end

