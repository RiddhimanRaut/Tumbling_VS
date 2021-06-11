function LG=getinteraction_ellipse3(cam,Z,truedepth,Zact,z, a, b, alpha,xd,yd)

KK=cam.K;
px = KK(1,1);
py = KK(2,2);
v0=KK(1,3);
u0=KK(2,3);
Zact=([Zact' Zact'])';

if(truedepth)
    Zarr=Zact(:);
else
    Zarr=Z*ones(8,1);
end
delta=0.01;

a=a+delta;%for matrix conditioning
b=b+delta;%for matrix conditioning

x0=z(1);
x1=z(1)-a*cos(alpha);
x2=z(1)+a*cos(alpha);
x3=z(1)+b*cos(alpha+pi/2);
x4=z(1)-b*cos(alpha+pi/2);
y0=z(2);
y1=z(2)-a*sin(alpha);
y2=z(2)+a*sin(alpha);
y3=z(2)+b*sin(alpha+pi/2);
y4=z(2)-b*sin(alpha+pi/2);


%s=[x0 x1 x2 x3 x4];
s=[x0; y0; x2; y2; x3; y3];
s=s(:);

s_len=length(s);
% plotellipse(z, a, b, alpha)
% hold on
% scatter([x1(1) x2(1) x3(1) x4(1)], [x1(2) x2(2) x3(2) x4(2)],20,'r','filled')


Ls=zeros(5,6);

for m=1:2:s_len
    x = (s(m) - px)/u0;
    y = (s(m+1) - py)/v0;
    
    %Zinv =  1/Z;
    Zinv =  1/Zarr(m);
    
    Ls(m,1) =  -Zinv;
    Ls(m,2) =  0;
    Ls(m,3) =  x*Zinv;
    Ls(m,4) =  x*y;
    Ls(m,5) =  -(1+x^2);
    Ls(m,6) =  y;
    

    Zinv =  1/Zarr(m+1);
    
    Ls(m+1,1) =  0;
    Ls(m+1,2) =  -Zinv;
    Ls(m+1,3) =  y*Zinv;
    Ls(m+1,4) = 1+y^2;
    Ls(m+1,5) = -x*y;
    Ls(m+1,6)  = -x;
    
end

L_x0=Ls(1,:);
L_y0=Ls(2,:);
L_x2=Ls(3,:);
L_y2=Ls(4,:);
L_x3=Ls(5,:);
L_y3=Ls(6,:);


T1=-(y2-y0)/((x2-x0)^2+(y2-y0)^2);
T2=(x2-x0)/((x2-x0)^2+(y2-y0)^2);

L_alpha=T1*(L_x2-L_x0)+T2*(L_y2-L_y0);
La=(x2-x0)*(L_x2-L_x0)/a +  (y2-y0)*(L_y2-L_y0)/a;
Lb=(x3-x0)*(L_x3-L_x0)/b +  (y3-y0)*(L_y3-L_y0)/b;


C=cos(alpha);
S=sin(alpha);

G1=(C*(xd-x0)/a)+(S*(yd-y0)/b);
G2=(S*(xd-x0)/a)-(C*(yd-y0)/b);


%LG1= G2*L_alpha-C*(x0-xd)*La/(2*a^2)-S*(x0-xd)*Lb/(2*b^2)+C*L_x0/a+S*L_y0/b;
%LG2=-G1*L_alpha+S*(x0-xd)*La/(2*a^2)-C*(x0-xd)*Lb/(2*b^2)-S*L_x0/a+C*L_y0/b;



LG1= +C*L_x0/a+S*L_y0/b;
LG2= -S*L_x0/a+C*L_y0/b;

LG=[LG1;LG2];

end
