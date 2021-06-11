function plot_velocity(vc_history, error_history)
shape = size(vc_history);
vels = shape(2);
total_iter = shape(1);
figure(3);
gcf;
xlabel('iterations');
ylabel('Velocity');
for i = 1:vels
    plot([1:total_iter],vc_history(:,i)');
    hold on;
end
legend('vel-x','vel-y','vel-z','omega-x','omega-y','omega-z')
figure(4)
plot([1:total_iter],error_history);
end