%���飺label_count
%�����������������Ԫ�ؼ�����Ӧ�������±�
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
target1_ID = max(1, 2);  % �����ص�����Ŀ������ı��
target1_pixelnum = max(1, 1);  % �����ص�����Ŀ���������������ص�ĸ���
label_count(max(1, 2)) = 0;% �Ѻ����ص�����Ŀ����������ص�������㣬����Ѱ�����ص�ζ��Ŀ������
max = [label_count(1),1];  % ���¶�max���г�ʼ��
for i = 2:1:Len
    if(max(1, 1) >= label_count(i))
    else
        max = [label_count(i),i];
    end
end
target2_ID = max(1, 2);  % �����ص�����Ŀ������ı��
target2_pixelnum = max(1, 1);  % �����ص�����Ŀ���������������ص�ĸ���
end