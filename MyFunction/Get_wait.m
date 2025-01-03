function wait=Get_wait(wait,j)

for i=1:size(wait)
    if  wait(i)==j
       break 
    end
    if  wait(i)==0
        wait(i)=j;
    end
    
end

end
