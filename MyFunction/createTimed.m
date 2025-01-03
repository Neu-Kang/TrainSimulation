function timeD=createTimed(Tr,Plat,TimeSchedule)
H=Plat.DepartInter;
timeD=[(Tr.M-1)*H zeros(1,Plat.N-1)];
for i=2:Plat.N
    timeD(i)=TimeSchedule.BeginTimeTable(1,i)-Plat.DepartInter;
end


end