%%----------------------------------------------------------------
%           Extract JPEG Frames from Video Input - GUI Version
%               calls the Function images_From_Video.m
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

clc;clear;close all;
build_GUI();
function build_GUI()
set(0,'units','centimeters');
screen = get(0,'Screensize');
width = 15.5;
height = 13.1;
h.fig = figure('name','Images from Video','NumberTitle','off','Toolbar','none','Menubar', 'none','NumberTitle','Off','units','centimeters','position',[(screen(3)-width)/2,(screen(4)-height)/2,width,height]);
        
uicontrol('style','text','String','File Options:',                  'units','centimeters','position',[0.3 11.9   5 1],'HorizontalAlignment','left','FontWeight','bold');
uicontrol('style','text','String','Source Video:',                  'units','centimeters','position',[0.3 11.3 2.5 1],'HorizontalAlignment','left');
h.details = uicontrol('style','text','String','File Details:',                  'units','centimeters','position',[0.3 10.7 10 1],'HorizontalAlignment','left');
uicontrol('style','text','String','Where to save the images?',      'units','centimeters','position',[0.3 10.1 6 1],  'HorizontalAlignment','left');
uicontrol('style','text','String','Save Path:',                     'units','centimeters','position',[0.3 9.5 2.5 1],'HorizontalAlignment','left');
uicontrol('style','text','String','Select Output Image Resolution:','units','centimeters','position',[0.3 3.5 5 0.5],'HorizontalAlignment','left','FontWeight','bold');
uicontrol('style','text','String','Quality:',                       'units','centimeters','position',[1.3 4.5 5 0.5],'HorizontalAlignment','left');
uicontrol('style','text','String','Select Output Image Quality:',   'units','centimeters','position',[0.3 5   5 0.5],'HorizontalAlignment','left','FontWeight','bold');
uicontrol('style','text','String','Stride:',                        'units','centimeters','position',[1.3 6   5 0.5],'HorizontalAlignment','left');
uicontrol('style','text','String','Save only every xth frame',      'units','centimeters','position',[6   6   5 0.5],'HorizontalAlignment','left');
uicontrol('style','text','String','Select Stride:',                 'units','centimeters','position',[0.3 6.5 5 0.5],'HorizontalAlignment','left','FontWeight','bold');
uicontrol('style','text','String','Calibration Options:',           'units','centimeters','position',[0.3 9   5 0.5],'HorizontalAlignment','left','FontWeight','bold');
uicontrol('style','text','String','Choose file:',                   'units','centimeters','position',[0.6 7.4 2.5 0.5],'HorizontalAlignment','left');

h.path_source          = uicontrol('style','edit',       'units','centimeters','position',[2.5 11.9 10.5  0.45 ]);
h.path_save            = uicontrol('style','edit',       'units','centimeters','position',[2.5 10.1 10.5  0.45 ],'enable','off');
h.save_folder_radio(1) = uicontrol('Style','radiobutton','Units','centimeters','Position',[5   10.75  4    0.34  ],'String','Create a Subfolder','Value',1);
h.save_folder_radio(2) = uicontrol('Style','radiobutton','Units','centimeters','Position',[9   10.75  4    0.34  ],'String','Manual Selection','Value',0);                       
h.calibrate            = uicontrol('style','checkbox',   'units','centimeters','position',[1.3  8.5 15    0.5  ],'string','Calibrate Images (1920x1080|4096x2160|3840x2160 Video calibration files for Phaton4Pro available)','Value',1);
h.calibrate_other      = uicontrol('style','checkbox',   'units','centimeters','position',[1.3  8   15    0.5  ],'string','Got dedicated Calibration camera parameters file','Value',0);
h.run                  = uicontrol('style','pushbutton', 'units','centimeters','position',[1.3 0.35  2    0.55 ],'String','Run');       
h.quality              = uicontrol('style','edit',       'units','centimeters','position',[2.5 4.6   1.5  0.45 ],'String',95);
h.stride               = uicontrol('style','edit',       'units','centimeters','position',[2.5 6.1   1.5  0.45 ],'String',10);
h.calibrate_file       = uicontrol('style','edit',       'units','centimeters','position',[2.5  7.45 10.5 0.45],'enable','off');
h.browse_calibrate     = uicontrol('style','pushbutton', 'units','centimeters','position',[13.3 7.4 2 0.55],     'String','Browse','enable','off','callback',{@browse_calibrate, h});                
h.browse_source        = uicontrol('style','pushbutton', 'units','centimeters','position',[13.3 11.85 2 0.55],    'String','Browse');
h.browse_save          = uicontrol('style','pushbutton' ,'units','centimeters','position',[13.3 10.05 2 0.55],    'String','Browse','enable','off'); 
h.update               = uicontrol('style','text',       'units','centimeters','position',[5 0.3 10 0.5],'String','');

h.resolution(1) = uicontrol('Style','radiobutton','Units','centimeters','Position',[1.3 1   4 0.5],'Value',1,'String','Original');                       
h.resolution(2) = uicontrol('Style','radiobutton','Units','centimeters','Position',[1.3 1.5 4 0.5],'Value',0,'String','FHD  ( 1920 X 1080 )');
h.resolution(3) = uicontrol('Style','radiobutton','Units','centimeters','Position',[1.3 2   4 0.5],'Value',0,'String','4K     ( 3840 X 2160 )');
h.resolution(4) = uicontrol('Style','radiobutton','Units','centimeters','Position',[1.3 2.5 4 0.5],'Value',0,'String','4K     ( 4096 X 2160 )');
h.resolution(5) = uicontrol('Style','radiobutton','Units','centimeters','Position',[1.3 3   4 0.5],'Value',0,'String','Manual                  X');
h.manual_width  = uicontrol('Style','edit','Units','centimeters','enable','off','Position',[3   3.05 1 0.45]);
h.manual_height = uicontrol('Style','edit','Units','centimeters','enable','off','Position',[4.5 3.05 1 0.45]);

set(h.calibrate,'callback',{@calibrate,h});
set(h.calibrate_other,'callback',{@calibrate_other,h});
set(h.resolution(1),'callback',{@resolutionradio,h});
set(h.resolution(2),'callback',{@resolutionradio,h});
set(h.resolution(3),'callback',{@resolutionradio,h});
set(h.resolution(4),'callback',{@resolutionradio,h});
set(h.resolution(5),'callback',{@resolutionradio,h});
set(h.save_folder_radio(1),'Callback', {@myRadio1,h});
set(h.save_folder_radio(2),'Callback', {@myRadio2,h});
set(h.browse_source,'callback',{@browse_source, h});
set(h.browse_save,'callback',{@browse_save,h});
set(h.manual_width, 'callback',{@resolution_manual_width, h});
set(h.manual_height,'callback',{@resolution_manual_height,h});
set(h.run,'callback',{@run, h});
end
function setsave(~,~,h)
global shuttleVideo
if exist(get(h.path_source,'string'),'file')
    [filepath,filename,~] = fileparts(get(h.path_source,'string'));
    if get(h.calibrate,'value') 
        partialsubfolder = strcat(filepath,'\',filename,'_images_calib');
    else
        partialsubfolder = strcat(filepath,'\',filename,'_images');
    end 
    resolution = shuttleVideo.width;
    if get(h.resolution(1),'Value')
        SubFolder = [partialsubfolder,'_',num2str(resolution)];
    elseif get(h.resolution(2),'Value') && resolution ~= 1920
        SubFolder = [partialsubfolder,'_1920_resized'];
    elseif get(h.resolution(3),'Value') && resolution ~= 3840
        SubFolder = [partialsubfolder,'_3840_resized'];
    elseif get(h.resolution(4),'Value') && resolution ~= 4096
        SubFolder = [partialsubfolder,'_4096_resized'];
    elseif get(h.resolution(5),'Value') && resolution ~= str2num(get(h.manual_width,'string')) 
        SubFolder = [partialsubfolder,'_',get(h.manual_width,'string'),'_resized'];
    else
        SubFolder = [partialsubfolder,'_',num2str(resolution)];
    end
    set(h.path_save, 'string', SubFolder);
end

end
function resolution_manual_width(~,~,h)
if isnan(str2double(get(h.manual_width,'string')))
    set(h.manual_width,'string','')
    msgbox ('Please Enter a number');
else   
    setsave(0,0,h);
end
end
function resolution_manual_height(~,~,h)
if isnan(str2double(get(h.manual_height,'string')))
    set(h.manual_height,'string','')
    msgbox ('Please Enter a number'); 
end
end
function browse_calibrate(~,~,h)
[sourcefile, sourcepath]=uigetfile({'*.mat'},'Select the Calibration file');
    set(h.calibrate_file,'string',[sourcepath, sourcefile]);
end
function resolutionradio(temp,event,h)

if event.Source.Value == 1
    for i = 1:length(h.resolution)
        if ~strcmp(event.Source.String,get(h.resolution(i),'string'))
            set(h.resolution(i),'Value',0);
        end
    end
else 
    set(temp,'Value',1);
end
if get(h.resolution(5),'Value')
    set(h.manual_width ,'enable','on');
    set(h.manual_height,'enable','on');
else
    set(h.manual_width ,'enable','off');
    set(h.manual_height,'enable','off');
    set(h.manual_width ,'string','');
    set(h.manual_height,'string','');
    setsave(0,0,h);
end
end
function calibrate(~,~,h)
    if ~get(h.calibrate,'value')
        set(h.calibrate_other,'value',0);
        set(h.calibrate_file,'string','');
        set(h.calibrate_file,'enable','off');
        set(h.browse_calibrate,'enable','off');
    end
    setsave(0,0,h);
end
function calibrate_other(~,~,h)
    if get(h.calibrate_other,'Value')
        set(h.calibrate,'value',1)
        set(h.calibrate_file,'enable','on');
        set(h.browse_calibrate,'enable','on');
    else
        set(h.calibrate_file,'string','');
        set(h.calibrate_file,'enable','off');
        set(h.browse_calibrate,'enable','off');        
    end
end
function myRadio1(~,~,h)

set(h.save_folder_radio(2),'value',0)
set(h.save_folder_radio(1),'value',1)
set(h.path_save, 'enable','off');
set(h.browse_save, 'enable','off');
setsave(0,0,h);

end
function myRadio2(~,~,h)

set(h.save_folder_radio(1),'value',0)
set(h.save_folder_radio(2),'value',1)
set(h.path_save, 'enable','on','string','');
set(h.browse_save, 'enable','on');

end
function browse_source(~,~,h)
global shuttleVideo
[sourcefile, sourcepath] = uigetfile({'*.mp4;*.avi;*.mov;*.MOV;*.MP4;*.AVI'},'Select the Video');
set(h.path_source, 'String',[sourcepath sourcefile]);
if exist(get(h.path_source,'string'), 'file')
	shuttleVideo = VideoReader(get(h.path_source,'string'));
    hours   = floor(shuttleVideo.Duration/3600);
    minutes = floor((shuttleVideo.Duration - hours * 3600)/60);
    seconds = floor(shuttleVideo.Duration - hours * 3600 - minutes*60);
    if hours<10
        hours = strcat('0',num2str(hours));
    else
        hours = num2str(hours);
    end
    if minutes<10
        minutes = strcat('0',num2str(minutes));
    else
        minutes = num2str(minutes);
    end
    if seconds<10
        seconds = strcat('0',num2str(seconds));
    else
        seconds = num2str(seconds);
    end
    set(h.details,'string', strcat('File Details:            Duration:      ',hours,':',minutes,':',seconds,'      FrameRate:      ',num2str(round(shuttleVideo.FrameRate)),'      No of Frames:      ',num2str(round(shuttleVideo.FrameRate * shuttleVideo.Duration))));
end
setsave(0,0,h);
end
function browse_save(~,~,h)

savepath = uigetdir('','Select the directory to save images');
set(h.path_save, 'String',savepath);

end
function run(~,~,h)
global shuttleVideo
sourcedirectory = get(h.path_source,'string');
savedirectory = get(h.path_save,'string');
set(h.fig, 'pointer', 'watch')
drawnow;

flag=1;
    if ~exist(sourcedirectory, 'file')
        set(h.path_source,'string','*Missing*');
        flag=0;
    elseif get(h.calibrate,'Value') && ~get(h.calibrate_other,'Value')
        if ~(shuttleVideo.width==1920 || shuttleVideo.width==4096 || shuttleVideo.width==3840)
            set(h.calibrate,'Value',0)
            waitfor(msgbox('Calibration cannot be done as the calibration file is Not available for the selected Video'));
        end
    end
    if ~exist(savedirectory, 'dir')
        if get(h.save_folder_radio(1),'value')&&flag==1
            mkdir(savedirectory)
        else
            if ~get(h.save_folder_radio(1),'value')
                set(h.path_save,'string','*Missing*');
            end
            flag=0;
        end
    end
    if get(h.calibrate_other,'Value')&&~exist(get(h.calibrate_file,'string'), 'file')
        set(h.calibrate_file,'string','*Missing*');
        flag=0;
    end
    if get(h.resolution(5),'value')
        if isempty(get(h.manual_width,'string'))
            set(h.manual_width,'string','*Missing*');
            flag=0;
        end
        if isempty(get(h.manual_height,'string'))
            set(h.manual_height,'string','*Missing*');
            flag=0;
        end 
    end
    
    if flag==1
        images_From_Video(0,0,0,0,0,0,h)
        msgbox('Done');
    else
        msgbox('Please fill the missing details');
    end
    
set(h.fig, 'pointer', 'arrow')
end
