%%remove buffer variables if it works without them, useless computation
close all
clear variables


% clear act
arduinoObj = serialport("COM10",9600); %%read objectno
%prompt='sampling frequency in numerals?';
%frequency=input(prompt);
%timespace=round(1/frequency,3,"significant");

% act=serialport("COM10", 9600);  

%%intialization variables%%
buff=30;%number of samples in buffer
plot_size=100; %%number of samples in a plot
%d=input('duration required? ','s');
%dur=str2double(d);hh
% dur=30; %%duration of record

configureTerminator(arduinoObj,"CR/LF"); %%terminator read
flush(arduinoObj); %%clear object each run to reset port
figure;
fig = uifigure("Position",[1000, 500, 200, 50]);
arduinoObj.UserData = struct("Data_buffer",[],"bufferlim",buff,"RAWData",[],"Data",[],"Time",[],"timer",0,"prevtime",0,"plotsize",plot_size,"firstime",0,"end",0,"fig",fig,"Count",0);

btn = uibutton(fig,'push', 'Text','Stop','Position',[100, 25, 100, 25],...
               'ButtonPushedFcn', @(btn,event) ender(arduinoObj));

configureCallback(arduinoObj,"terminator",@plotData); %%function to call each time data is detected


function plotData(src, ~)

% Read the ASCII data from the serialport object.
data = readline(src); 
val=str2double(data);
% Convert the string data to numeric type and save it in the UserData
% property of the serialport object.
if(isnan(val)) %%reset for next read if value is trash
    return
end  
val2=val;
%val2=round((((val/15925248)-0.5)*2*2.4/3.5),4,"significant");
%%val2=round(val*3.3/4096,4,'significant');

%src.UserData.DataBuffer(end+1) = val2; %% complete data array into which value is stored
%src.UserData.DataBuffer(end+1) = val; %% data buffer array into which value is stored, buffer_dur*fs in size
if src.UserData.firstime==0
    tic;
    src.UserData.firstime=1;
end
toc
temptime=toc;
src.UserData.Time(end+1)=temptime; %% time array for storing value
src.UserData.Data(end+1)=val2;
% src.UserData.Data_buffer(end+1)=val2;

src.UserData.Count = src.UserData.Count + 1; %% helps figure the starting time for the data to be plotted
%temptime_buffer=(length(src.UserData.DataBuffer))*src.UserData.timegap;%%buffer timeline

%src.UserData.ValBuffer(end+1)=temptime; %% time buffer array for mapping
%deltime=temptime-src.UserData.prevtime; %%difference in time since previous operation
%src.UserData.prevtime=temptime; %% save new time

%src.UserData.timer=src.UserData.timer+src.UserData.timegap; %% timer to figure out buffer

%if(temptime_buffer>src.UserData.timelim_filt)
  %  emg_filtered=highpass(src.UserData.DataBuffer,10,src.UserData.fs); %filter the buffer
    %src.UserData.RAWData=cat(2,src.UserData.RAWData, src.UserData.DataBuffer); %append raw data
    %disp("buffer length:");
    %disp(length(src.UserData.DataBuffer));
 %   src.UserData.Data=cat(2,src.UserData.Data,emg_filtered); %append filtered data
 %   src.UserData.Data_rms=cat(2,src.UserData.Data_rms,rms(emg_filtered)); %find rms and add the value
 %   src.UserData.DataBuffer=[]; 
    %disp("time length:");
    %disp(length(src.UserData.Time));
    %disp("data length:");
    %disp(length(src.UserData.RAWData));
    %disp(length(src.UserData.Data));

%     if src.UserData.firstime==0 && temptime>7 %threshold calculation
%        % src.UserData.Baseline=src.UserData.Data_rms;
%         src.UserData.firstime=1;
%         %src.UserData.Baseline_avg=mean(src.UserData.Baseline);
%         %src.UserData.Baseline_sd=std(src.UserData.Baseline);
%         %disp("threshold calculated:");
%         %disp(src.UserData.Baseline_avg);
%         %disp(src.UserData.Baseline_sd);
%     end

%     if src.UserData.firstime==1 && (src.UserData.Data_rms(end)>(src.UserData.Baseline_avg+3*src.UserData.Baseline_sd)||src.UserData.Data_rms(end)<(src.UserData.Baseline_avg-3*src.UserData.Baseline_sd))
%         disp("1");
%     elseif src.UserData.firstime==1
%         disp("0");
%     end

%     if src.UserData.timer >= src.UserData.timelim  %%only entered after buffer duration
%         src.UserData.timer=0; %%reset buffer clock timer
if mod(src.UserData.Count,src.UserData.bufferlim)==0
%     data_filt=lowpass(src.UserData.Data_buffer,10,90);
%     src.UserData.Data_buffer=[];
%     src.UserData.Data=cat(2,src.UserData.Data, data_filt); %append raw data
        if src.UserData.Count < src.UserData.plotsize %%plot after number of samples attained
           plot(src.UserData.Time,src.UserData.Data);
          %  plot(src.UserData.Time,src.UserData.Data);
            title('Force Data');
            xlabel('Time(s)');
            ylabel('Force(g)');
             ylim([-1000 1000])
            %xlim([0 7])    
            drawnow;
        end
        if src.UserData.Count > src.UserData.plotsize %%plot after number of samples attained
           plot(src.UserData.Time(end-int32(src.UserData.plotsize+1):end),src.UserData.Data(end-int32(src.UserData.plotsize+1):end));
            %plot(src.UserData.Time(end-int32(src.UserData.plotsize+1):end),src.UserData.Data(end-int32(src.UserData.plotsize+1):end));
            title('Force Data');
            xlabel('Time(s)');
            ylabel('Force(g)');           
            ylim([-1000 1000])
            xlim([src.UserData.Time(end-int32(src.UserData.plotsize+1)) src.UserData.Time(end)])    
            drawnow;
        end
 end
%end

    if src.UserData.end==1
        toc
        configureCallback(src, "off");
        saver(src)
    
    end
end

function saver(src)

%     b=struct("Data",[],"Time",[]);
        AllData=src.UserData;
        Data=AllData.Data(2:end);
%         RAWData=AllData.RAWData(2:end);
        Time=AllData.Time(2:end);
        
        
        

    prompt='file name?';
    fname=input(prompt,"s");
%     fname=join(["Data\",x],"");
    save(['D:\Documents\MATLAB\Load Cell\' fname],'Data', 'Time');
    delete(src.UserData.fig);
end

function ender(src)
    disp("ended")
    src.UserData.end=1;
    
end

