function keepvar=iterativeData(Tr,Plat,keepvar)
TimeEr=keepvar.TimeEr5;
Er=[];
for i=1:Tr.TrCirAll
   Er=[Er sum(TimeEr(:,1+Plat.N*(i-1):Plat.N*i), 2)];
end
keepvar.TimeErColumnSum5=Er;
Er=sum(Er,1);
keepvar.TimeErLineSum5=Er;
end