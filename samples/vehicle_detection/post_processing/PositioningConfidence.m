%%----------------------------------------------------------------
%                PositioningConfidence Plot (bins)
%           Only relevant for Benchmark with Reference Sensor
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Sanchez, E., Kruber F.
% All rights reserved.
%----------------------------------------------------------------

function [binvalues,Diagram]=PositioningConfidence(ErrorValues,bins,VideoID)

if ~exist('bins')
    bins=100;
end
binvalues=zeros(bins,3);
for i=1:bins
    binvalues(i,1)=(max(ErrorValues)/bins)*i;
end
ErrorValues=abs(ErrorValues);
ErrorValues=sortrows(ErrorValues,1);

currentbin=1;
for i=1:size(ErrorValues,1)-1
    if ErrorValues(i)>binvalues(currentbin,1)
        binvalues(currentbin,2)=i-1;
        currentbin=currentbin+1;
    end
end
binvalues(min([currentbin,bins-1]):end,2)=binvalues(currentbin-1,2);
binvalues(end,2)=size(ErrorValues,1);
binvalues(:,3)=binvalues(:,2)./size(ErrorValues,1);
Diagram=figure('Name','Percentage of measurements smaller than abscissa','units','normalized','outerposition',[0 0 1 1]);
plot(binvalues(:,1),binvalues(:,3)*100,'b','LineWidth',2)
hold on
xlabel('Error')
ylabel('%')
grid on
set(gca,'FontSize',20)
FullTitle=strcat('Percentage of measurements smaller than abscissa',' for Video -',num2str(VideoID));
title(FullTitle)
ylim([0 100])

end