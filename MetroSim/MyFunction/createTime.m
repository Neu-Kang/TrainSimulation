function y=createTime(x,n)
%�������ܣ�������վ�������ڻ�ͼ��11 22 33������ͷ���
cir=0; %��ǰȦ��
num=0; %��ǰ���
anum=0; %ʵ�����
for i=1:x/2
    num=mod(i,n);
    if num==0
        anum=2;
    else
        if num<=n/2+1
            anum=num;
        else
            anum=n/2+1-mod(num,n/2+1) ;
        end
    end
    y(2*(i-1)+1)=anum;
    y(2*i)=anum;
    
end