clear;
clc;
startup_rvc;
hold off;
lambda=0.003;
mu=0.0001;
my_cube = Cube_points(2,[5,7,15],0.1,0.2,0.3);
centre_x = sum(my_cube(1,:))/8;
centre_y = sum(my_cube(2,:))/8;
centre_z = sum(my_cube(3,:))/8;
centre = [centre_x,centre_y,centre_z];
cam1 = CentralCamera('default');
cam2 = CentralCamera('default');
cam2.T = cam2.T*SE3.Rx(0.5)*SE3([15,6,-15]);
HTM = eye(4);
p = cam1.project(my_cube);
xy_des = cam2.project(my_cube);

frames = 15;
d = 0.0001*ones(6,1);
e = ones(6,1);
while abs(e)>0
    Z = 1;
    frames_stack_x = p(1,:);
    frames_stack_y = p(2,:);
    for c = 1:frames
        my_cube = AxelRot(my_cube,20,[0 1 0],centre);
        p = cam1.project(my_cube);
        frames_stack_x = [frames_stack_x; p(1,:)];
        frames_stack_y = [frames_stack_y; p(2,:)];
        figure(1);
        plot(p(1,:),p(2,:),'.');
        axis([ 0 1024 0 1024 ]);
        pause(0.1)
    end
    
    hold off;
    ellipse_params = [];
    for i = 1:8
        im_x = frames_stack_x(:,i);
        im_y = frames_stack_y(:,i);
        im = [im_x, im_y]';
        [z, a, b, alpha] = fitellipse(im,'linear','constraint','trace');
        ellipse_params = [ellipse_params;[z', a, b, alpha, xy_des(1,i), xy_des(2,i)]];
        figure(1);
        plotellipse(z, a, b, alpha, 'b--');
        plot(xy_des(1,:),xy_des(2,:),'.');
        axis([ 0 1024 0 1024 ]);
        hold on;
    end
    error = error_ellipse(ellipse_params);
    Lsd = getinteraction(ellipse_params,cam1);
    Hsd = Lsd'*Lsd;
    diagHsd = eye(size(Hsd,1)).*Hsd;
    H = inv((mu * diagHsd) + Hsd);
    e =  H * Lsd' *error;
    vc =  lambda*e;
    %e = abs(e)
    %vc = [1 1 1 0.1 0.1 0.1]'
    Td = SE3(trnorm(delta2tr(vc)));
    cam1.T = cam1.T*Td;
    pause(0.1);
    
end

    
