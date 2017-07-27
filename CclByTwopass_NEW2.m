%Two-pass 连通区域标记算法（4邻接定义规则）（由图像左上角向图像右下角方向扫描）
%输入：image是一副二值图像
%输出：output_args1 = label;    与图像image同维的一个矩阵，存放每一个像素的标号
%           output_args2 = array;     等价对数组
%           output_args3 = nlabel;    对标号进行计数的一个变量，输出该变量以便分析和调试代码
%           output_args4 = label_count;  用于统计每个标号对应的像素点的个数
function [ output_args1, output_args2, output_args3, output_args4 ] = CclByTwopass_NEW2( image )
array = ones(30000,1,'uint16');   %等价对数组
[m, n] = size(image);
label = ones(m, n,'uint16');   %与图像image同维的一个矩阵，每个元素的值表示与image同位置处的像素的标号
nlabel = 2;
%% ONE-PASS
% 先单独处理第一行第一列的元素
if(image(1,1) == 0) % 证明是前景点，而不是背景点
    label(1,1) = nlabel;
    nlabel = nlabel + 1;
end
% 再处理第一行其余元素
for j = 2:1:n
    if(image(1, j) == 0) % 证明是前景点，而不是背景点
        if (image(1, j) == image(1, j-1))
            label(1, j) = label(1, j-1);
        else
            label(1, j) = nlabel;
            nlabel = nlabel + 1;
        end
    end
end
% 再处理第一列的元素
for i = 2:1:m
    if(image(i, j) == 0)  % 证明是前景点，而不是背景点
        if (image(i, 1) == image(i-1, 1))
            label(i, 1) = label(i-1, 1);
        else
            label(i, 1) = nlabel;
            nlabel = nlabel + 1;
        end
    end
end
% 然后再处理剩余的像素点 %
for i = 2:1:m
    for j = 2:1:n
        if(image(i,j) == 0)  % 证明是前景点，而不是背景点
            if (image(i, j) == image(i, j-1) || image(i, j) == image(i-1, j))
               if (image(i, j-1) ~= image(i-1, j))
                   if (image(i, j) == image(i, j-1))
                       label(i, j) = label(i, j-1);
                   else
                       label(i, j) = label(i-1, j);
                   end
               else
                   if (label(i, j-1) == label(i-1, j))
                       label(i, j) = label(i-1, j);
                   else
                       if (label(i, j-1) > label(i-1, j))
                           label(i, j) = label(i-1, j);
                           array(label(i,j-1)) = label(i-1, j); % setunion
                           label(i, j-1) = label(i-1, j);  %
                       else
                           label(i, j) = label(i, j-1);
                           array(label(i-1, j)) = label(i, j-1);
                       end 
                   end
               end
            else
                label(i, j) = nlabel;
                nlabel = nlabel + 1;
            end 
        end
    end
end
%% 使用setfind函数对等价对树进行路径压缩（Path Compression）
for i = 2:1:nlabel
    if (array(i) > 1)
        array(i) = setfind(array(i), array);
    end
end
%% TWO-PASS 
for i = 1:1:m
    for j = 1:1:n
        while(array(label(i, j)) > 1)
            label(i, j) = array(label(i, j));
        end
        
    end
end
 %%统计每个标号对应的像素点的个数（准确地讲，这一部分代码已经不属于Two-pass CCL算法了，这是为后续对靶标的提取和识别做的准备工作）
label_count = zeros(nlabel, 1, 'uint16'); % 用于统计每个标号对应的像素点的个数
for i = 1:1:m
    for j = 1:1:n
        if(label(i, j) ~= 1)
            label_count(label(i, j)) = label_count(label(i, j)) + 1;
        end
    end
end
output_args1 = label;
output_args2 = array;
output_args3 = nlabel;
output_args4 = label_count;
end