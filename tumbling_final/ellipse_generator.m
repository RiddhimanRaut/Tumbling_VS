function [ellipse_mat,error] = ellipse_generator(xy1, xy1_arr, xydes)
    ellipse_mat = [];
    delta=0.01;
    for i = 1:size(xy1,2)
       temp(:,:) = xy1_arr(:,i,:);
       pause(0.1)
       try
            %fit ellipses on all keypoints
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
       ellipse_mat = [ellipse_mat;[z(:,i)', a(i), b(i), alpha(i), ellipse_error(i)]];
    end
end