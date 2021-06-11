function cubeplotter(cube)
myfaces = [1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8;
        1 2 3 4;
        5 6 7 8];
patch('Faces',myfaces,'Vertices',cube,'FaceColor','g');
end