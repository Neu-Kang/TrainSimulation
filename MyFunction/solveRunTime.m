function Runtime=solveRunTime(timetableAfter)
%计算列车运行时间
[Tnum,Pnum]=size(timetableAfter);
for i=1:Tnum
   z=1;
   for j=2:2:Pnum
      if j==Pnum
         break 
      end
      Runtime(i,z)=timetableAfter(i,j+1)-timetableAfter(i,j);
      z=z+1;
   end
end
end

    
    