function [y,ylast,judge]=checkOver(Tr,ylast)
%����Ƿ���Խ��judge 1Ϊ����Խ��
    y1=sortline(Tr);
    if ~isequal(y1,ylast)
        judge=1;
    else
        judge=0;
    end
    y=y1;
