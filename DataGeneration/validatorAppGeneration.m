appFile = "Validator.m";
buildResults = compiler.build.standaloneApplication(appFile, 'AdditionalFiles', {'walk_test_2.txt', 'tracker_file.m', 'T_Results.mat'});
compiler.package.installer(buildResults);
system("Validatorinstaller/MyAppInstaller.exe", "AgreeToLicense", 'yes');
