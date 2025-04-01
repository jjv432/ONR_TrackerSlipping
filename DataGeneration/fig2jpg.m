clc; close all; format compact

addpath("Figures")
addpath("Images")

names = erase(string(ls("Figures")), " ");
names = names(3:end);

for i = 1:numel(names)

    openfig(names(i));
    saveas(gcf, "Images/" + erase(names(i), ".fig") + ".jpg");
    close

end
