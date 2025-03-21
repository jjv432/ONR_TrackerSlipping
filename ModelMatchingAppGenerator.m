addpath("ModelFunctionsFiles");
addpath("ModelFunctionsFiles\autoSlide\")
addpath("ModelFunctionsFiles\autoStick\")

appFile = "unifiedfriction_simulation.m"; % File that you're making the .exe off of
buildResults = compiler.build.standaloneApplication(appFile, 'AdditionalFiles', {'ModelFunctionsFiles/walk_test_2.txt', 'tracker_file.m', 'T_Results.mat'}); % any files that the main script references
compiler.package.installer(buildResults); % creates the installer
system("unifiedfriction_simulationinstaller/MyAppInstaller.exe", "AgreeToLicense", 'yes'); % runs the installer


%{

When this is done, a folder called "ModelMatchingstandaloneApplication" is
generated. Go to this directory, and execute 'ModelMatching.exe'

You'll need to install the matlab compiler package for this to work

%}
