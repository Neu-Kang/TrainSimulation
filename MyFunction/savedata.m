function savedata(obj)
%�������ܣ����浽����matlabԴ�ļ�zhuodu.txt
    out=fread(obj,50,'char'); %��ȡ����
    fid=fopen('zhuodu.txt','a+'); %��zhuodu.txt
    fprintf(fid,'%s',out); %���ļ���д������
    fclose(fid); %�ر��ļ�
    d=textscan('G1.txt','%s'); %��ȡ����
    plot(d); %����ͼ��
    disp('saveok!'); %���saveok��