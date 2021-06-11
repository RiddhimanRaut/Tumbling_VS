function cube = Cube_points(side,initial_corner,rx,ry,rz)
    vert = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1]';
    cube = side*vert;
    H = SE3.Rx(rx)*SE3.Ry(ry)*SE3.Rz(rz)*SE3(initial_corner);
    cube = H*cube;
    cube = cube';
    
end