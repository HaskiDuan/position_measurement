%根据椭圆的中心点、长轴、短轴及方向角画椭圆
%输入：（Xcenter,Ycenter）为中心点坐标，LongAxis长轴，ShortAxis短轴，Angle方向角
% 示例：
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