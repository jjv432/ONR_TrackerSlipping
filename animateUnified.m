function animateUnified(t,q,FPS)

t_anim = t(1):1/FPS:t(end);



l_anim = interp1(t,q(:,1),t_anim);  % leg length
qB_anim = interp1(t,q(:,3),t_anim); % leg angle 
x_anim = interp1(t,q(:,5),t_anim);  % foot position

% Define hip
R = 0.05;                           % radius (just for visualization)

theta = linspace(0,2*pi,50);
hx = R*cos(theta); hy = R*sin(theta);


for iter = 1:numel(t_anim)

    clf
    hold on
    % Foot position
    x_f(iter) = x_anim(iter); y_f(iter) = 0;

    % hip center position

    x_h = x_anim(iter) + l_anim(iter)*cos(qB_anim(iter));
    y_h = l_anim(iter)*sin(qB_anim(iter));
    
    xx_h = x_h + hx;
    yy_h = y_h + hy;

    plot([-1 1],[0 0],'Color', 'k','LineWidth', 3)
    plot([x_f x_h],[y_f y_h],'Color', 'k','LineWidth', 3)
    fill(xx_h,yy_h,'b');
   

    hold off
    axis equal
   
    axis([-1 1 -0.1 0.6])

    pause(5/FPS)
end
