clear
clc
%Cube target
cube = Cube_points(2,[3,7,5],0.5,0.2,0.3); %[x1 y1 z1; x2 y2 z2;...]
T_OC=[eul2rotm([0 0 0]*pi/180) [-7; 5; 12]]; %initial projection matrix - cMw --- 3 Rows, 4 Columns
Tdes_OC=[eul2rotm([10,-7,-15]*pi/180) [-20; 5; 20]]; %desired projection matrix --- 3 Rows, 4 Columns
[xyinit,xydes,kp_init,kp_des,cam] = vs_init(T_OC, Tdes_OC,cube);
kp_curr=kp_init;
iter=1;
iter1 = 1;
frames = 6;
vc_history = [];
error_history = [];
figure(1);
figure(2);
% figure(5);
while(1)
   for iter = 1:frames
       centre = mean(cube);
       rotation = AxelRot(cube',20,[ 0 1 0 ], centre); %Arg1 - 3xN, Arg2 - deg, Arg3-dir, Arg4-1x3 centre pt
       cube = rotation'; %new cube pts
%        figure(5)
%        clf;
%        axis([-10 10 -10 10 -10 10])
%        cubeplotter(cube);
       T_OC_curr=pinv([cam.T;0 0 0 1]);
       Opt_pts = T_OC_curr*[cube';ones(1,size(cube',2))];
       Opt_pts = Opt_pts(1:3,:);
       xt = cam.K*Opt_pts;
       x1=[xt(1,:)./xt(3,:)];
       y1=[xt(2,:)./xt(3,:)];
       xy1=([x1' y1'])';
       figure(2);
       plot(x1,y1,'.');
       plot(xydes(1,:),xydes(2,:),'ro');
       axis([-1024 1024 -1024 1024])
       hold on;
       xy1_arr(:,:,iter)=xy1;
       kp_curr=xy1(:);
       iter=iter+1;
   end
   clf;
   hold on;
   [mat_ellipse,error] = ellipse_generator(xy1, xy1_arr, xydes);
   z = [mat_ellipse(:,1),mat_ellipse(:,2)]';
   a = mat_ellipse(:,3);
   b = mat_ellipse(:,4);
   alpha = mat_ellipse(:,5);
   ellipse_error = mat_ellipse(:,6);
   err = norm(ellipse_error);
   error_history = [error_history,err];
   Z=abs(cam.T(3,4));
   Lsd=[];
   for i0=1:size(error,2)
        Lsd=[Lsd;getinteraction_ellipse3(cam,Z,1,xt(3,:),z(:,i0), a(i0), b(i0), alpha(i0),xy1(1,i0),xy1(2,i0))];
   end
   vc = control_algo(Lsd,error);
   vc_history = [vc_history;vc'];
   Tdiff=[eul2rotm([vc(6),vc(5),vc(4)]) [vc(1); vc(2); vc(3)]]; %rot_ZYX
   temp1=(([cam.T;[0 0 0 1]])*[Tdiff;[0 0 0 1]]);
   cam.T(1:3,:)=temp1(1:3,:);
   fprintf('iter1=%d,ellipse_err=%.2f, vc=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n',iter1,err,vc(1),vc(2),vc(3),vc(4),vc(5),vc(6));
   iter1 = iter1 +1;
   if(norm(vc)<0.01)break;end
end
plot_velocity(vc_history,error_history);
