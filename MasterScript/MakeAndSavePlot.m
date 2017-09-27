function MakeAndSavePlot( x, xLabel, y, yLabel, graphTitle, filename )
% MakeAndSavePlot() Summary of this function goes here
% Input:    x:
%           xLabel:
%           y:
%           yLabel:
%           graphTitle:
%           filename:
% Output:   NONE. Note that the plot is shown and saved as a png file in
%           the current working directory.
% Date:     24 September 2017

% Assert that filename ends in .png

% Make plot
plot(x, y)
title(graphTitle) % Add title
xlabel(xLabel) % Add label for x-axis
ylabel(yLabel) % Add label for y-axis

% Save plot
saveas(gcf, filename)

end

