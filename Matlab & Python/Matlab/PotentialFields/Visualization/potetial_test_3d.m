clear all
clc
[X,Y] = meshgrid(-30:0.05:30,-30:0.05:30);
x=-30:0.5:30;
y=x';
Z1 =10*exp(-(Y-6).^2/8);
Z2 = 0.5*0.1*(1./(Y+20).^2)+0.5*0.1*(1./(Y-20).^2);
Z3 =1*exp(-(Y-0.5).^2/8)+1*exp(-(Y-11.5).^2/8)+ 0.5*1*(1./(Y+3).^2)+0.5*1*(1./(Y-13).^2);
Z4 =20*exp(-0.8*(sqrt((Y).^2+((X-4).^2))-0.4));
Z5 =0.05*((30-X).^2+(0-Y).^2).^1.5;
Z6 =-20/(2*pi)*log(sqrt((Y+10).^2+((X-4).^2)))+20/(2*pi)*log(sqrt((Y+5).^2+((X-2).^2)));
Z7=-10*(X*cos(pi/100)+Y*sin(pi/100));
Z10=0.01*(X.^2+Y.^2+(X-10).^2+(Y-10).^2);
% for k=1:1
%     Z10=Z10+2*exp(-(X-k).^2/4-(Y-k).^2/40);
% %Z10=1*(2*exp(-X.^2/3-Y.^2/3)+2*exp(-(X-2).^2/3-(Y-2).^2/3)+2*exp(-(X-4).^2/3-(Y-4).^2/3)+2*exp(-(X-6).^2/3-(Y-6).^2/3));
% end
% for i=1:20
% Z8=0.05*((X-25).^2+(Y-10).^2).^(0.5*2);
% Z9=20*(1./(((X-i).^2+(Y-i).^2).^0.5)-1/(10));
% surf(X,Y,Z9);%412
% hold on;
% 
% end
surf(X,Y,Z10);
shading interp;
%  [f_x,f_y]=gradient(-Z10);
% % surf(X,Y,Z4+Z1+Z2);%412
% % shading interp;
% % hold on
% % figure
% % %contour(x,y,z4+z1+z2)
% figure
%  hold on
%  quiver(X,Y,f_x,f_y)
%  hold off
