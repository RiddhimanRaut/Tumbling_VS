function [xyinit,xydes,kp_init,kp_des,cam] = vs_init(T_OC, Tdes_OC,cube)
    cam.K=[800 0 320; 0 800 240; 0 0 1];
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
end