clc; clearvars -except Data; close all; format compact;

addpath("SlippingFunctions");
addpath("TrackerFiles");
addpath("Figures");
%{

TO-DO

There's somthing about the 0p5 Hz trial that is causing stride 4 to have
one less index than all of the other strides... curently have a not so
great fix for it

!! Make the struct capable of holding multiple of the same frequency, then
average between those !!
- Make a 'field_combiner' function that goes through all of the fields,
checks if the frequencies (and weight) are the same, then averages these
together and stores it in a "mean_X_Hz" field
- Can use the 'fields' function

Go through with FFT function so that I can get the best frequency estimate

%}

%% Asking user if they want to read from a .json
% Change this so that it allows user to enter the name
readFromJson = input("Do you want to read Data from a .mat file (y/n)? ", 's');
if readFromJson == 'y'
    clear Data
    new_mat_name = input("Enter the name of the file, including the extension: ", 's');
    Data = load(new_mat_name);

end

%% Only doing some functions if Data doesn't exist in the workspace

if ~exist('Data', 'var')
    [Data, FullFileNames, AbrevFileNames] = makeNewDataStruct();
    Data.FullFileNames = FullFileNames;
    Data.AbrevFileNames = AbrevFileNames;
else
    FullFileNames = Data.FullFileNames;
    AbrevFileNames = Data.AbrevFileNames;

end


%% Call trial_finder
%{
Trial_finder is helpful as it allows the user to see what each of their
data plots looks like. From this, they can determine how many trials they
would like to remove from the end of the data.  By doing this, noise/
sloppy trials can be removed.
%}

trial_finder_answer = input("Do you want to look and/or adjust the trials (y/n)? ", 's');

if trial_finder_answer == 'y'
    Data = trial_finder(Data, FullFileNames, AbrevFileNames);
end

%% Calling frequency_statistics
%{
This function gets statistical data for all of the frequencies
independently in the data struct.
%}

plotting_answer = input("Do you want to see the statistics for the trials (they will be updated regardless) (y/n)? ", 's');
if plotting_answer == 'y'
    plotBool = 1;
else
    plotBool = 0;
end
Data = frequency_statistics(Data, FullFileNames, AbrevFileNames, plotBool);

%% Asking the user if they want to save these results
save_answer = input("Want to save these results to a .mat file(y/n)? ", 's');

if save_answer == 'y'
    save("Data.mat", '-struct', 'Data');
end

%% Cleaning up
clc; clearvars -except Data
