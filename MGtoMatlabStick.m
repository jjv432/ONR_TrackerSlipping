% Parsing
% Read the equations from the text file
fileID = fopen('MGEqnsStick.txt', 'r');
equations = textscan(fileID, '%s', 'Delimiter', '\n');
fclose(fileID);

% Convert cell array to string array
equations = string(equations{1});

% Define the desired replacements
old_vars = {"l''", "l'", "qB''", "qB'", "stretch"};
new_vars = {'ddl', 'dl', 'ddqB', 'dqB','(l-ldes)'};

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




%% Turn into symbolic equations
syms l qB 
syms dl dqB 
syms ddl ddqB 

syms mH mQ ldes k g
syms Tau T_w

syms Ff Fn

syms Fdh_x Fdh_y Fdl_x Fdl_y Fbh_y


symbolic_equations = str2sym(equations)


params = getParams();

%expr_numeric = subs(symbolic_equations, [IAz,IBz], [params.IAz, params.IBz]);

expr_numeric = subs(symbolic_equations, [mH, mQ, k,  g], [params.mH, params.mQ, params.k, params.g]);


qq_aug = [ddl ddqB Ff Fn];
Maug = jacobian(expr_numeric, qq_aug);
f = -subs(expr_numeric,{ddl; ddqB; Ff; Fn},{0;0;0;0});  

Maug = formula(Maug)
f = formula(f)

matlabFunction(Maug,'vars',{[l;qB]},'file',['autoStick' filesep 'M_aug_func_stick']);

Fluid_forces = [Fdh_x; Fdh_y; Fdl_x; Fdl_y; Fbh_y; T_w];
matlabFunction(f,'vars',{[l;qB;Tau;ldes],[dl;dqB], Fluid_forces},'file',['autoStick' filesep 'f_func_stick']);




