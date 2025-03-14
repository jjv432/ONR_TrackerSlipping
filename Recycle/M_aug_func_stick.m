function Maug = M_aug_func_stick(in1)
%M_aug_func_stick
%    Maug = M_aug_func_stick(IN1)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    24-Feb-2025 14:50:14

l = in1(1,:);
qB = in1(2,:);
t2 = cos(qB);
t3 = sin(qB);
Maug = reshape([t2.*(9.4e+1./5.0),t3.*(9.4e+1./5.0),9.4e+1./5.0,0.0,l.*t3.*(-9.4e+1./5.0),l.*t2.*(9.4e+1./5.0),0.0,l.^2.*(9.4e+1./5.0),-1.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0],[4,4]);
end
