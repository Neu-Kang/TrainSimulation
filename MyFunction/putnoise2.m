function TrStay=putnoise2(i,j,k,time,Tr,Plat,noise)
TrStay=time;
switch noise
    case 3
        if ismember(i,2:Tr.M)&&(j==3||j==7)
            t1=Plat.TimeA(Tr.TrPass(i))-Plat.TimeD(Tr.TrPass(i));
            t2=(t1+Plat.Tmin)/(1-Plat.a*Plat.noiseck);
            TrStay=t2-t1;
        end
        if i==1&&Tr.TrCir(i)~=0&&(j==3||j==7)
            t1=Plat.TimeA(Tr.TrPass(i))-Plat.TimeD(Tr.TrPass(i));
            t2=(t1+Plat.Tmin)/(1-Plat.a*Plat.noiseck);
            TrStay=t2-t1;         
        end
        if i==1&&Tr.TrCir(i)==0&&(j==3||j==7)
            t1=Plat.DAH;
            TrStay=(Plat.a*Plat.noiseck*t1+Tr.Tmin)/(1-Plat.a*Plat.noiseck);
        end
end