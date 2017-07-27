%数组：label_count
%输出：最大的两个数组元素及所对应的数组下标
function [target1_ID, target1_pixelnum, target2_ID, target2_pixelnum] = GMS(label_count)
Len = length(label_count);
max = zeros(1, 2, 'uint16');
max = [label_count(1),1];
for i = 2:1:Len
    if(max(1, 1) >= label_count(i))
    else
        max = [label_count(i),i];
    end
end
target1_ID = max(1, 2);  % 含像素点最多的目标区域的编号
target1_pixelnum = max(1, 1);  % 含像素点最多的目标区域所含的像素点的个数
label_count(max(1, 2)) = 0;% 把含像素点最多的目标区域的像素点个数置零，方便寻找像素点次多的目标区域
max = [label_count(1),1];  % 重新对max进行初始化
for i = 2:1:Len
    if(max(1, 1) >= label_count(i))
    else
        max = [label_count(i),i];
    end
end
target2_ID = max(1, 2);  % 含像素点最多的目标区域的编号
target2_pixelnum = max(1, 1);  % 含像素点最多的目标区域所含的像素点的个数
end