function Npast=GetNpast(time,loop,TrInter)
overvalue=mod(time,loop);
loopnum=fix(time/loop);
overnum=0;
for i=1:size(TrInter,2)
   if sum(TrInter(1:i))>=overvalue
       overnum=i-1;
       break
   else
       continue
   end
end
Npast=loopnum*size(TrInter,2)+1+overnum; %第一圈多一个站，其余站正常
end