function y=createvarmatrix(m,n)
%�������ܣ����������ά������һ����������
y= sym(zeros(m,n));
for i=1:m
    for j=1:n
        cmd = sprintf('sym(''X%i%i'')',i,j);
        y(i,j) = eval(cmd);
    end
end