function fig = loadFigureSettings(datasetName, doSaveFigure)

if (nargin == 1)
    doSaveFigure = 0;
end

isSubplot = 2; % x 1 id subplot 2x2 used, x 2 if single plot.

fig.yLabel1 = 'repeatability %';
fig.yLabel2 = '#correspondences';
fig.yLabel3 = 'matching score %';
fig.yLabel4 = '#correct matches';

fig.style = { 'mx:',  'ks-.', 'bo-', 'r*--' };

if ~doSaveFigure
    % Legend properties: set Title, axis size
    fig.tSize = 10;
    % Set Legend size
    fig.lSize = 8;%20
    % Set FontUnit size
    fig.uSize = 15;
    % Set line width
    fig.lWidth = 2;
    % Set marker size
    fig.mSize = 7;
else
    % Legend properties: set Title, axis size
    fig.tSize = 10*isSubplot;
    % Set Legend size
    fig.lSize = 12*isSubplot;%20
    % Set FontUnit size
    fig.uSize = 15*isSubplot;
    % Set line width
    fig.lWidth = 2*isSubplot;
    % Set marker size
    fig.mSize = 7*isSubplot;
end

switch datasetName
    case 'bark'
        fig.xValues = [ 1.2 1.8 2.5 3 4 ];
        fig.xLabel = 'scale change';
    case 'bikes'
        fig.xValues = [ 2 3 4 5 6 ];
        fig.xLabel = 'blur (increasing)';
    case 'boat'
        fig.xValues = [ 1.13 1.37 1.9 2.35 2.8 ];
        fig.xLabel = 'scale change';
    case 'graf'
        fig.xValues = [ 20 30 40 50 60 ];
        fig.xLabel = 'viewpoint angle';
    case 'leuven'
        fig.xValues = [ 2 3 4 5 6 ];
        fig.xLabel = 'light change (decreasing)';
    case 'trees'
        fig.xValues = [ 2 3 4 5 6 ];
        fig.xLabel = 'blur (increasing)';
    case 'ubc'
        fig.xValues = [ 60 80 90 95 98 ];
        fig.xLabel = 'JPEG compression %';
    case 'wall'
        fig.xValues = [ 20 30 40 50 60 ];
        fig.xLabel = 'viewpoint angle';
    otherwise
        error('Error, dataset not found.');
end