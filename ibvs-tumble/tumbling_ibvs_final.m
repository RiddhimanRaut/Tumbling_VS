clear
clc

%K matrix for camera
cam.K=[800 0 320; 0 800 240; 0 0 1];


%Cube target
cube = Cube_points(2,[-3,7,12],0.1,-0.2,0.5); %[x1 y1 z1; x2 y2 z2;...]

curr_roll=0;
curr_yaw=0;
curr_pitch=0;

T_OC=[eul2rotm([0 0 0]*pi/180) [7; 5; 6]]; %initial projection matrix - cMw --- 3 Rows, 4 Columns
Tdes_OC=[eul2rotm([10,3,5]*pi/180) [-20; 5; 6]]; %desired projection matrix --- 3 Rows, 4 Columns


cam.T= [(T_OC(1:3,1:3))' -(T_OC(1:3,1:3))'*(T_OC(1:3,4))]; %wMc = [R' -R'T; 0 1] initial camera pose --- 3 Rows, 4 Columns
cam_des.T= [(Tdes_OC(1:3,1:3))' -(Tdes_OC(1:3,1:3))'*(Tdes_OC(1:3,4))]; %wMc = [R' -R'T; 0 1] desired camera pose --- 3 Rows, 4 Columns

%initial camera projections
xt_init = cam.K*T_OC*[cube';ones(1,size(cube',2))]; %3x8 
xinit=[xt_init(1,:)./xt_init(3,:)];
yinit=[xt_init(2,:)./xt_init(3,:)];
xyinit=([xinit' yinit'])'; %---2 rows, 8 columns
kp_init=xyinit(:); %[x1;y1;x2;y2;...] 16x1

%final camera projections
xt_des=cam.K*Tdes_OC(1:3,:)*[cube';ones(1,size(cube',2))]; 
xdes =[xt_des(1,:)./xt_des(3,:)];
ydes =[xt_des(2,:)./xt_des(3,:)];
xydes =([xdes' ydes'])';
kp_des = xydes(:);

kp_curr=kp_init;
iter=1;
iter1 = 1;

%delta is for numerical stability 
delta=0.01;
%delta=eps;

%lambda and mu are gradient descent parameters
lambda=0.0003;
mu=0.001;
frames = 20;
figure(1);
figure(2);
while(1)
   for iter = 1:frames
       centre = mean(cube);
       rotation = AxelRot(cube',20,[ 0 1 0 ], centre); %Arg1 - 3xN, Arg2 - deg, Arg3-dir, Arg4-1x3 centre pt
       cube = rotation'; %new cube pts
       T_OC_curr=pinv([cam.T;0 0 0 1]);
       Opt_pts = T_OC_curr*[cube';ones(1,size(cube',2))];
       Opt_pts = Opt_pts(1:3,:);
       xt = cam.K*Opt_pts;
       x1=[xt(1,:)./xt(3,:)];
       y1=[xt(2,:)./xt(3,:)];
       xy1=([x1' y1'])';
       figure(2);
       plot(x1,y1,'.');
       plot(xdes,ydes,'ro');
       axis([-1024 1024 -1024 1024])
       hold on;
       xy1_arr(:,:,iter)=xy1;
       kp_curr=xy1(:);
       iter=iter+1;
   end
   clf;
   hold on;
   for i = 1:size(xy1,2)
       temp(:,:) = xy1_arr(:,i,:);
       pause(0.1)
       try
            %fit ellepses on all keypoints
            [z(:,i), a(i), b(i), alpha(i)] = fitellipse(temp, 'linear', 'constraint','trace');
            figure(1)
            plotellipse(z(:,i), a(i), b(i), alpha(i))
            hold on;
            Q = [cos(alpha(i)) -sin(alpha(i));sin(alpha(i)) cos(alpha(i))];
            A =Q'* (xydes(:,i)-z(:,i))./[a(i)+1;b(i)+1];
            alpha(i)=mod(alpha(i)+pi,pi);
            
            %errors along x and y axis
            error1(i)=(cos(alpha(i))*(z(1,i)-xydes(1,i))/(a(i)+delta)) + (sin(alpha(i))*(z(2,i)-xydes(2,i))/(b(i)+delta));
            error2(i)=-(sin(alpha(i))*(z(1,i)-xydes(1,i))/(a(i)+delta)) + (cos(alpha(i))*(z(2,i)-xydes(2,i))/(b(i)+delta));
            error(:,i)=[error1(i);error2(i)];
            
            A1 =Q'* (xydes(:,i)-z(:,i))./[a(i)+eps;b(i)+eps];
            
            %ellipse error
            ellipse_error(i)=norm(A1)-1;
            
            
        catch
            %if the fitting function is not able to fit ellipse.
            %Mostly it becomes degenerate when major or minor axis tends to zero.
            %then ellipse could be estimated using straingt line.

            disp('Catch')
            p1=polyfit(temp(1,:),temp(2,:),1);
            p2=polyfit(temp(2,:),temp(1,:),1);
            if(norm(p1(1))<norm(p2(1)))
                p=p1;
                a(i) = max(abs(max(temp(1,:)) - min(temp(1,:)))/2,1);
                b(i) = max(abs(max(temp(2,:)) - min(temp(2,:)))/2,1);
                alpha(i) = p1(1);
            else
                p=p2;
                b(i) = max(abs(max(temp(2,:)) - min(temp(2,:)))/2,1);
                a(i) = max(abs(max(temp(1,:)) - min(temp(1,:)))/2,1);
                alpha(i) = p2(1);
            end
            
            z(:,i) = mean(temp,2);
            set(gca,'Ydir','reverse')
            plotellipse(z(:,i), a(i), b(i), alpha(i))
            Q = [cos(alpha(i)) -sin(alpha(i));sin(alpha(i)) cos(alpha(i))];
            A =Q'* (xydes(:,i)-z(:,i))./[a(i)+1;b(i)+1];
            alpha(i)=mod(alpha(i)+pi,pi);
            error1(i)=(cos(alpha(i))*(z(1,i)-xydes(1,i))/a(i)+delta) + (sin(alpha(i))*(z(2,i)-xydes(2,i))/b(i)+delta);
            error2(i)=-(sin(alpha(i))*(z(1,i)-xydes(1,i))/a(i)+delta) + (cos(alpha(i))*(z(2,i)-xydes(2,i))/b(i)+delta);
            error(:,i)=[error1(i);error2(i)];
            A1 =Q'* (xydes(:,i)-z(:,i))./[a(i)+eps;b(i)+eps];
            ellipse_error(i)=norm(A1)-1;
            
       end
       
   end
   Z=abs(cam.T(3,4));
   Lsd=[];
   for i0=1:size(error,2)
        Lsd=[Lsd;getinteraction_ellipse3(cam,Z,1,xt(3,:),z(:,i0), a(i0), b(i0), alpha(i0),xy1(1,i0),xy1(2,i0))];
   end
   if(norm(pinv(Lsd))>1000)
        Hsd = Lsd'*Lsd;
        diagHsd = eye(size(Hsd,1)).*Hsd;
        H = inv((mu * diagHsd) + Hsd);
        e =  H * Lsd' *error(:);
        vc = - lambda*e;
   else
        vc = - lambda*pinv(Lsd)*error(:);
   end
   vc;
   Tdiff=[eul2rotm([vc(6),vc(5),vc(4)]) [vc(1); vc(2); vc(3)]]; %rot_ZYX

   temp1=(([cam.T;[0 0 0 1]])*[Tdiff;[0 0 0 1]]);
   cam.T(1:3,:)=temp1(1:3,:);
   fprintf('iter1=%d,ellipse_err=%.2f, vc=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n',iter1,norm(ellipse_error),vc(1),vc(2),vc(3),vc(4),vc(5),vc(6));
   iter1 = iter1 +1;
   if(norm(vc)<0.005)break;end
end



