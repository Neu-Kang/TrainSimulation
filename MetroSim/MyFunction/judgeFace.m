function ans=judgeFace(tur,range)
%�������ܣ��ж��Ƿ񵽴�ֽ��棬�����ﷵ��1�����򷵻�0
    turspeed=tur(length(tur))-tur(length(tur)-1); %��ּ���仯��
    if turspeed>=range(1)&&turspeed<=range(2) %�ж��Ƿ��ڷ�Χ�ڼ�⵽�ֽ���
        ans=1;
    else
        ans=0;
    end
    