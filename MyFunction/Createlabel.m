function lable=Createlabel(length,pnum)
%����վ̨������pnum���ܾ���վ̨��length�������������ᣨHM1,HM2,HM3��
    forname=['A','B','C'];
    lable={};
    for i=1:length
        if i==27
            y=0;
        end
        a=floor((i-1)/pnum)+1;%number of circle
        b=mod2(i,pnum);%number of plat
        lable(size(lable,1)+1,:)={[num2str(a),forname(b)]};
%         lable(size(lable,1),:)=cell(lable(size(lable,1),:));
    end
end