% --- Update axis labels in a .fig and save (Times New Roman, italic, 24) ---

inFig  = "C:\Users\Jeff DesRoches\OneDrive - Worcester Polytechnic Institute (wpi.edu)\Poudel, Prakash's files - ACSCP fire\ForestFire\ACC2026Paper\fig\Lawnmower_Sensor_Coverage.fig";
outFig = inFig;   % or overwrite by setting outFig = inFig

% Open the .fig invisibly and get the figure handle
fig = openfig(inFig);

% Find axes to label (skip legends/colorbars)
ax = findall(fig,'Type','axes');
ax = ax(~arrayfun(@(a) isa(a,'matlab.graphics.illustration.Legend') || ...
                     isa(a,'matlab.graphics.illustration.ColorBar'), ax));

% LaTeX gives clean subscripts x_1, x_2
for k = 1:numel(ax)
    xlabel(ax(k), '$x_1$', ...
        'Interpreter','latex', ...
        'FontName','Times New Roman', ...
        'FontSize',24);  % 'FontAngle','italic'...      
        

    ylabel(ax(k), '$x_2$', ...
        'Interpreter','latex', ...
        'FontName','Times New Roman', ...
        'FontSize',24);  %'FontAngle','italic' ...
end

% Save updated .fig (and optionally a raster/vector export too)
savefig(fig, outFig);

[p,f] = fileparts(inFig);
pngFile = fullfile(p, f + ".png");
exportgraphics(fig, pngFile, 'Resolution', 300);

