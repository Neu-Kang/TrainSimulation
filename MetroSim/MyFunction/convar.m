function y=convar(a,b)
%�˺������Խ�����������Ķ���ʽ�˷�������һ������������������
%aΪ��ֵ����ž���bͬ��
na=length(a)-1;
nb=length(b)-1;
for i=1:na+1
    for j=1:nb+1
        if(i==1||(i~=1&&j==nb+1))
           y(1,i-1+j-1+1)=a(1,i)*b(1,j); 
        else
            y(1,i-1+j-1+1)=a(1,i)*b(1,j)+y(1,i-1+j-1+1);
        end
    end
end


