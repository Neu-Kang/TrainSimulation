function TrWait=Delete_wait(TrWait)
for i=1:size(TrWait)
    if TrWait(i)==0
       break 
    else
        TrWait(i)=TrWait(i+1);
        
    end
end

end