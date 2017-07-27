%输入：向量vector
%输出；norm（向量vector的2范数）
function norm=norm2(vector) 
    L = length(vector);
    norm = 0;
    for i = 1:1:L
        norm = norm + vector(i)^2;
    end
    norm = sqrt(norm);
end