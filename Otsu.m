%% Otsu法求阈值
% 输入image是一副灰度图像
function threshold=Otsu(image)
[rows,columns]=size(image);
total_pixel=rows*columns; % 图像总共的像素点数
% 先求灰度柱状图
histogram=zeros(1,256);
variance=zeros(1,256);
for i=1:1:rows
    for j=1:1:columns
        histogram(1,image(i,j)+1)=histogram(1,image(i,j)+1)+1;
    end
end

for T=0:1:255
    num1=0;
    num2=0;
    forward_value=0;
    back_value=0;
    for i=0:1:255
        if(i<=T)
            num1=num1+histogram(1,i+1);
            forward_value=forward_value+histogram(1,i+1)*i;
        else
            num2=num2+histogram(1,i+1);
            back_value=back_value+histogram(1,i+1)*i;
        end
    end
    P1=num1/total_pixel;
    P2=num2/total_pixel;
    forward_average=forward_value/num1;
    back_average=back_value/num2;
    variance(1,T+1)=P1*P2*(forward_average-back_average)*(forward_average-back_average);
end
max_value=max(variance);
for i=0:1:255
    if(max_value==variance(1,i+1))
        threshold=i;
    end      
end

%figure;
%stem([0:1:255],histogram,'linewidth',2);
%xlim([0 255]);
%set(gca,'xTick',[0:50:250]);
%set(gca,'xTickLabel',{'0' '50' '100' '150' '200' '250'});
%set(gca,'FontSize',18);
%set(get(gca,'XLabel'),'FontSize',22);
%set(get(gca,'YLabel'),'FontSize',22);
%xlabel('灰度强度');
%ylabel('像素点个数');
%title('灰度强度直方图（加权平均）');%
%title('灰度强度直方图（直接求和取平均）');
