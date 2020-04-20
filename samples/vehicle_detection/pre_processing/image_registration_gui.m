%%----------------------------------------------------------------
%   Perform Image Registration based on Matlab Functions with GUI
%      calls the Function image_registration.m
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

clc;clear;close all;
h.fig = figure('name','Image Registration','NumberTitle','off','Toolbar','none',...
                'Menubar', 'none','NumberTitle','Off');
set(gcf,'position',[500,500,600,100])
h.path_text_source = uicontrol('style','text','String','Source Images Path',...
                         'units','normalized','position',[0.02 0.6 0.20 0.25]);
h.path_text_save   = uicontrol('style','text','String','Save Images Path  ',...
                         'units','normalized','position',[0.02 0.3 0.20 0.25]);
h.keyframe_text    = uicontrol('style','text','String','Key Frame/Image   ',...
                         'units','normalized','position',[0.02 0 0.20 0.25]); 
h.quality_text    = uicontrol('style','text','String','Quality',...
                         'units','normalized','position',[0.28 0 0.20 0.25]);                      
h.update    = uicontrol('style','text','String','',...
                         'units','normalized','position',[0.68 0 0.3 0.25]);          
h.path_source = uicontrol('style','edit',...
                          'units','normalized','position',[0.22 0.64 0.6 0.25]);
h.path_save   = uicontrol('style','edit',...
                          'units','normalized','position',[0.22 0.34 0.6 0.25]);   
h.keyframe    = uicontrol('style','edit','String',1,...
                          'units','normalized','position',[0.22 0.05 0.1 0.25]);
h.quality    = uicontrol('style','edit','String',95,...
                          'units','normalized','position',[0.44 0.05 0.1 0.25]);
h.browse_source = uicontrol('style','pushbutton','String','Browse',...
                            'units','normalized','position',[0.84 0.65 0.12 0.25],...
                            'callback',{@browse_source, h});                
h.browse_save = uicontrol('style','pushbutton','String','Browse',...
                          'units','normalized','position',[0.84 0.35 0.12 0.25],...
                          'callback',{@browse_save, h});                
h.run = uicontrol('style','pushbutton','String','Run',...
                  'units','normalized','position',[0.56 0.05 0.12 0.25],...
                  'callback',{@run, h});
                 
function browse_source(~,~,h)

sourcepath = uigetdir('','Select the directory of distorted images');
set(h.path_source, 'String',sourcepath);

end
                 
          
function browse_save(~,~,h)

savepath = uigetdir('','Select the directory to save registered images');
set(h.path_save, 'String',savepath);

end
                 
                 
                 
function run(~,~,h)
sourcedirectory = get(h.path_source,'string');
savedirectory = get(h.path_save,'string');
set(h.fig, 'pointer', 'watch')
drawnow;

flag=1;
    if ~exist(sourcedirectory, 'dir')
        set(h.path_source,'string','*Missing*');
        flag=0;
    end
    if ~exist(savedirectory, 'dir')
        set(h.path_save,'string','*Missing*');
        flag=0;
    end
    if flag==1
        image_registration(0,0,0,0,h)
        msgbox('Done');
    end
    
set(h.fig, 'pointer', 'arrow')
end


