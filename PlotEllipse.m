%������Բ�����ĵ㡢���ᡢ���ἰ����ǻ���Բ
%���룺��Xcenter,Ycenter��Ϊ���ĵ����꣬LongAxis���ᣬShortAxis���ᣬAngle�����
% ʾ����
% PlotEllipse(0,0,3,5,pi/6)
function PlotEllipse(Xcenter,Ycenter,LongAxis,ShortAxis,Angle)

t1=0:.02:pi;
t2=pi:.02:2*pi;
z1=exp(i*t1);
z2=exp(i*t2);
z1=(LongAxis*real(z1)+i*ShortAxis*imag(z1))*exp(i*(-Angle));
z2=(LongAxis*real(z2)+i*ShortAxis*imag(z2))*exp(i*(-Angle));
z1=z1+Xcenter+Ycenter*i;
z2=z2+Xcenter+Ycenter*i;


plot(z1','g','linewidth',2);
plot(z2','g','linewidth',2);


%hold off
grid on