function noise=setnoisenum(num,M)
%�������ܣ���ָ�������г��������
switch num
    case 0
         noise=zeros(1,M)-1;
    case M
        noise=zeros(1,M);
    otherwise
        noise=[zeros(1,num),zeros(1,M-num)-1];
end


