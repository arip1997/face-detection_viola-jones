function varargout = facedetection(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @facedetection_OpeningFcn, ...
    'gui_OutputFcn',  @facedetection_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end
if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function facedetection_OpeningFcn(hObject, eventdata,  handles, varargin)
handles.output = hObject;
guidata(hObject, handles);
movegui(hObject, 'center');
clc
clear

function varargout = facedetection_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

reqToolboxes = {'Computer Vision System Toolbox', 'Image Processing Toolbox'};
info = ver;
s=size(info);

flg = zeros(size(reqToolboxes));
reqSize = size(reqToolboxes,2);

for i=1:s(2)
    for j=1:reqSize
        if( strcmpi(info(1,i).Name,reqToolboxes{1,j}) )
            flg(1,j)=1;
        end
    end
end
ret = prod(flg);

if ~ret
    error('detectFaceParts requires: Computer Vision System Toolbox and Image Processing Toolbox. Please install these toolboxes.');
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)

[filename,pathname]=uigetfile('*.*');
if ~isequal(filename,0)
    handles.data1 = imread(fullfile(pathname,filename));
    guidata(hObject,handles);
    axes(handles.axes1)
    cla reset
    imshow(handles.data1);
else
    return
end

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)

tic
X = handles.data1;
thresholdFace = 1;
thresholdParts = 1;
stdsize = 176;

nameDetector = {'LeftEye'; 'RightEye'; 'Mouth'; 'Nose'; };
mins = [[12 18]; [12 18]; [15 25]; [15 18]; ];

detector.stdsize = stdsize;
detector.detector = cell(5,1);
for k=1:4
    minSize = int32([stdsize/5 stdsize/5]);
    minSize = [max(minSize(1),mins(k,1)), max(minSize(2),mins(k,2))];
    detector.detector{k} = vision.CascadeObjectDetector(char(nameDetector(k)), 'MergeThreshold', thresholdParts, 'MinSize', minSize);
end

detector.detector{5} = vision.CascadeObjectDetector('FrontalFaceCART', 'MergeThreshold', thresholdFace);

bbox = step(detector.detector{5}, X);
bbsize = size(bbox);
partsNum = zeros(size(bbox,1),1);
stdsize = detector.stdsize;

for k=1:4
    if( k == 1 )
        region = [1,int32(stdsize*2/3); 1, int32(stdsize*2/3)];
    elseif( k == 2 )
        region = [int32(stdsize/3),stdsize; 1, int32(stdsize*2/3)];
    elseif( k == 3 )
        region = [1,stdsize; int32(stdsize/3), stdsize];
    elseif( k == 4 )
        region = [int32(stdsize/5),int32(stdsize*4/5); int32(stdsize/3),stdsize];
    else
        region = [1,stdsize;1,stdsize];
    end
    
    bb = zeros(bbsize);
    for i=1:size(bbox,1)
        XX = X(bbox(i,2):bbox(i,2)+bbox(i,4)-1,bbox(i,1):bbox(i,1)+bbox(i,3)-1,:);
        XX = imresize(XX,[stdsize, stdsize]);
        XX = XX(region(2,1):region(2,2),region(1,1):region(1,2),:);
        
        b = step(detector.detector{k},XX);
        
        if( size(b,1) > 0 )
            partsNum(i) = partsNum(i) + 1;
            
            if( k == 1 )
                b = sortrows(b,1);
            elseif( k == 2 )
                b = flipud(sortrows(b,1));
            elseif( k == 3 )
                b = flipud(sortrows(b,2));
            elseif( k == 4 )
                b = flipud(sortrows(b,3));
            end
            
            ratio = double(bbox(i,3)) / double(stdsize);
            b(1,1) = int32( ( b(1,1)-1 + region(1,1)-1 ) * ratio + 0.5 ) + bbox(i,1);
            b(1,2) = int32( ( b(1,2)-1 + region(2,1)-1 ) * ratio + 0.5 ) + bbox(i,2);
            b(1,3) = int32( b(1,3) * ratio + 0.5 );
            b(1,4) = int32( b(1,4) * ratio + 0.5 );
            
            bb(i,:) = b(1,:);
        end
    end
    bbox = [bbox,bb];
end

bbox = [bbox,partsNum];
bbox(partsNum<=2,:)=[];

face =  bbox(:,1: 4);
axes(handles.axes2)
cla reset
imshow(X);
hold on

[m, ~] = size(face);
for j = 1:m
    rectangle('Position',[face(j,1),face(j,2),face(j,3),face(j,4)],'EdgeColor','y','LineWidth',2);
end
hold off
toc


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = getframe(gca);
[filename2,pathname2] = uiputfile(...
    {'*.bmp','bitmap image (*.bmp)';
    '*.jpg','jpeg image(*.bmp)';
    '*.*','All file(*.*)'},...
    'Save Image');
if ~isequal(filename2,0)
    imwrite(img.cdata,fullfile(pathname2,filename2));
else
    return
end
