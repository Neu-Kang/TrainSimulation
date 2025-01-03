function k=SigmoidMatch(x,y)
%�������ܣ��Ը������ݽ�����ϲ����طֽ���б�ʷ�Χ,����ͼ��
    myfittype=fittype('a./(b+exp(-cx))',...
        'dependent',{'y'},'independent',{'x'},'coefficients',{'a','b','c'});
    %����sigmoid��ʶ����
    myfit=fit(x',y',myfittype);%���ݸ������ݽ��б�ʶ
    plot(myfit,x,y); %������ʶ������������ݵ�ͼ��
    d=diff(myfit,{'x'}); %�Ժ���������
    max_where=find(d==max(d));%Ѱ�ҵ�������λ��
    k1=d(max_where); %�������ֵ
    y1=y(max_where); %�������λ��yֵ
    x1=max_where; %�������λ��xֵ
    z=@(x)k1(x-x1)+y1; %�������ߺ���
    plot(z,myfit); %�����������ʶ����
    k=[k1-5 k1+5];%���طֽ���б�ʷ�Χ
    
