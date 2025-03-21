% Parsing
% Read the equations from the text file
% Using Continuous Friction Model
fileID = fopen('MGEqnsSlide.txt', 'r');
equations = textscan(fileID, '%s', 'Delimiter', '\n');
fclose(fileID);

% Convert cell array to string array
equations = string(equations{1});

% Define the desired replacements
old_vars = {"Ff", "l''", "l'", "qB''", "qB'", "stretch","x''", "x'" };
new_vars = {'-muk*Fn*dx/(abs(dx) + epsilonV)','ddl', 'dl', 'ddqB', 'dqB','(l-ldes)','ddx', 'dx'};

% Loop through each equation and perform replacements
for i = 1:numel(equations)
    eq = equations(i);
    for j = 1:numel(old_vars)
        eq = strrep(eq, old_vars{j}, new_vars{j});
    end
    equations(i) = eq;
end

% Display the modified equations
disp('Modified Equations:');
disp(equations);
%keyboard()



%% Turn into symbolic equations
syms x l qB 
syms dx dl dqB 
syms ddx ddl ddqB 

syms mH mQ ldes k g muk epsilonV
syms Tau T_w

syms Ff Fn

syms Fdh_x Fdh_y Fdl_x Fdl_y Fbh_y


symbolic_equations = str2sym(equations)



params = getParams();

%expr_numeric = subs(symbolic_equations, [IAz,IBz], [params.IAz, params.IBz]);

expr_numeric = subs(symbolic_equations, [mH, mQ, k,  g, epsilonV], [params.mH, params.mQ, params.k, params.g, params.epsilonV]);

%keyboard();
% stick
% qq_aug = [ddl ddqB Ff Fn];  
% Maug = jacobian(expr_numeric, qq_aug);
% f = -subs(expr_numeric,{ddl; ddqB; Ff; Fn},{0;0;0;0});  

% continuous friction
qq_aug = [ddl ddqB ddx Fn];  
Maug = jacobian(expr_numeric, qq_aug);
f = -subs(expr_numeric,{ddl; ddqB; ddx; Fn},{0;0;0;0});  


Maug = formula(Maug)
f = formula(f)

%matlabFunction(Maug,'vars',{[l;qB],[dx],[muk;epsilonV]},'file',['auto' filesep 'M_aug_func_slide']);
matlabFunction(Maug,'vars',{[l;qB],[dx],[muk]},'file',['autoSlide' filesep 'M_aug_func_slide']);

Fluid_forces = [Fdh_x; Fdh_y; Fdl_x; Fdl_y; Fbh_y; T_w];
matlabFunction(f,'vars',{[l;qB;Tau;ldes],[dl;dqB], Fluid_forces},'file',['autoSlide' filesep 'f_func_slide']);

keyboard();

%Fluid_forces = [Fbuoy_leg_y; Fdrag_leg_x; Fdrag_leg_y; Torque_water; Fbuoy_hip_y; Fdrag_hip_x; Fdrag_hip_y];
%matlabFunction(f,'vars',{[l;qA;qB;xF; Flin; Tau],[dl;dqA;dqB;dxF], Fluid_forces},'file',['auto' filesep 'f_func']);


