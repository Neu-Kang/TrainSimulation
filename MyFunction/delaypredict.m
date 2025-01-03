function [delay]=delaypredict(DAH,NaH,Tnum,M,delaytime,Tr,Plat)
%% 发车晚点预测程序
ckpoint=Plat.ckpoint;
noiseck=Plat.noiseck;
P=Plat.P;
Tmin=Plat.Tmin;
stayH=Plat.stayH;
H=Plat.H;
delaynow=delaytime; %初始晚点
delay=zeros(1,M); %晚点向量初始化
delay=zeros(1,Tr.M)
delay(Tnum)=delaynow;
timeFD=H-stayH-delaynow;
timeDD=(timeFD+Tmin)/(1-ckpoint*P);

for i=1:100
    
    if i<=2
       timeFD=1;
       timeDD=(timeFD+Tmin)/(1-noiseck*P);
       delayn=delaynow+timeDD-H;
       
    else
        z=(-1)^i;
        timeFD=H-stayH-delaynow;
        timeDD=(timeFD+Tmin)/(1-ckpoint*P);
        delayn=delaynow+timeDD-H;
        
    end
    if abs(delayn)<0.1
       break 
    end
    delay(Tnum+i)=ceil(delayn*10)/10;
    delaynow=delayn;

end

end