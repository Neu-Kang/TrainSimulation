function [ck,stayvar]=GetCk(ckpoint,ckd,N,stayvar)
if mod(stayvar.controller,stayvar.controller_num)==1 || stayvar.controller_num==1
    ck=roundn(ckpoint-ckd+2*rand(1,N)*ckd,-2);
    stayvar.ck=ck;
else
    ck=stayvar.ck;
end
end