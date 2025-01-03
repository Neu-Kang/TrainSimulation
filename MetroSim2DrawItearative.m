function MetroSim2DrawItearative(keep)

%% 迭代次数晚点变化
time=20;
delay_times=keep.TimeErLineSum5(1:time);

leng=size(delay_times,2);
iterations=1:leng;

figure;
plot(iterations, delay_times, '-o', 'LineWidth', 2);
xlabel('Iterative Times','FontSize',25);
ylabel('Total Train Delays at Each Iteration','FontSize',25);
grid on;

%% 

beginPlat=3;
endPlat=8;
beginPlatt=3;
endPlatt=8;

figure('name','Single platform Total Error Algorithm Comparison Diagram');
x=linspace(beginPlat,endPlat,endPlat-beginPlat+1);
y1=sum(keep.TimeEr2(:,x));
y2=sum(keep.TimeEr4(:,x));
y3=sum(keep.TimeEr5(:,x));
y4=sum(keep.TimeEr6(:,x));

bar(x,[y1',y2',y3',y4'])
set(gca,'XLim',[beginPlatt-1 endPlatt+1]);
set(gca,'XTick',[beginPlatt:1:endPlatt]);
set(gca,'XTickLabel',{'3(TJD)';'4(JLCBD)';'5(BH)';'6(JLCBU)';'7(TJU)';'8(WQU)';'9(BJN)';'10(WQD)';'11(TJD)';'12(JLCBD)';'13(BH)';'14(JLCBU)';'15(TJU)';'16(WQU)';'17(BJN)';'18(WQD)';'19(TJD)';'20(JLCBD)'},'FontSize',20);

lgd=legend('Feedback Control method','MPC method','IMPC methdod','MIP method');
lgd.FontSize=22;
xlabel('Station ID','FontSize',25);
ylabel('Total Train Delays at each Station','FontSize',25);





end