function MinRuntime=createRuntimeYou(minruntime,plat_num)
plat_Num=size(minruntime,2);
MinRuntime=[];
for i=1:plat_num-1
   j=mod(i,plat_Num);
   if j==0
       MinRuntime=[MinRuntime minruntime(end)];
   else
       MinRuntime=[MinRuntime minruntime(j)];
   end
    
end