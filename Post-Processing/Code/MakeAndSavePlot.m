function [] = MakeAndSavePlot( x, xLabel, y, yLabel, graphTitle, pathname, filename )
% Input:    x: Array of x values
%           xLabel: String
%           y: Array of y values
%           yLabel: String
%           graphTitle: String
%           filename: String ending in .png
% Output:   NONE. Note that the plot is shown and saved as a png file in
%           the current working directory.
% Author:   Victoria Skeggs
% Date:     24 September 2017

% Assert that filename ends in .png

% Make plot
plot(x, y);
xlim([0, x(end)]); % Set x-limits (can adjust to preference)
title(graphTitle); % Add title
xlabel(xLabel); % Add label for x-axis
ylabel(yLabel); % Add label for y-axis

% Save plot
saveas(gcf, fullfile(pathname, filename)); % Saves and overwrites existing image
end
