function TrInter=GetInter2(RunTime,H,stayH,stayHS)
length=size(RunTime,2);
for i=1:length
    if i==length/2||i==length
        TrInter(i)=RunTime(i)+stayHS;
    else
        TrInter(i)=RunTime(i)+stayH;
    end    
end
end