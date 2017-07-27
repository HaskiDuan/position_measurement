%输入：数组array
%输出：array中的局部极小值点
function output_args = NonMinSup(array)
local_min = zeros(20,1);
count = 1;
leng = length(array);
i = 2;
while( i <= leng-1 )
    if(array(i) < array(i+1))
        if(array(i) <= array(i-1))
           local_min(count,1) = i;
           count = count + 1;
        end
    else
        i = i+1;
        while( i < leng-1 && array(i)>=array(i+1))
            i = i + 1;
        end
        if ( i < leng - 1 )
           local_min(count,1) = i;
           count = count + 1; 
        end
    end
    i = i + 2;
end
output_args = local_min;
end