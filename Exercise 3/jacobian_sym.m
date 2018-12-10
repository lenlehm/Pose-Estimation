syms x y z tx ty tz;

%setting up vector field (function)
F = [x*y, cos(x*z), log(3*x*z*y)];

v = [x y z tx ty tz]; % partial derivatives for each column

J = jacobian(F,v); % calculate jacobian 

