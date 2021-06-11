function err_mat = error_ellipse(ellipse_params)
    param_matrix_shape = size(ellipse_params);
    rows = param_matrix_shape(1);
    err_mat = [];
    for i = 1:rows
        x0 = ellipse_params(i,1);
        y0 = ellipse_params(i,2);
        a = ellipse_params(i,3);
        b = ellipse_params(i,4);
        alpha = ellipse_params(i,5);
        xd = ellipse_params(i,6);
        yd = ellipse_params(i,7);
        C=cos(alpha);
        S=sin(alpha);
        e_x = (C*(xd-x0)/a)+(S*(yd-y0)/b);
        e_y = (-S*(xd-x0)/a)+(C*(yd-y0)/b);
        err_mat = [err_mat;[e_x;e_y]];
    end
    
end