%�鲢���㷨�еĲ鼯�㷨������Two-pass�㷨�У����ڶԵȼ۶�������·��ѹ��
function root = setfind(node, array)
if (array(node) <= 1)
    root = node;
else
    array(node) = setfind(array(node), array);
    root = array(node);
end