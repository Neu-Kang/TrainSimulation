function y=structToarray(x)
%�ɽ�solve���֮��ĵĽṹ��ת��Ϊ�������
fileds = fieldnames(x);
for i=1:length(fileds)
    k = fileds(i);
    key = k{1};
    value = x.(key);
    y(1,i)=value;
end