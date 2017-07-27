%查并集算法中的查集算法，用在Two-pass算法中，用于对等价对树进行路径压缩
function root = setfind(node, array)
if (array(node) <= 1)
    root = node;
else
    array(node) = setfind(array(node), array);
    root = array(node);
end