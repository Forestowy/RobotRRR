function varargout = RobotRRR(varargin)
% ROBOTRRR MATLAB code for RobotRRR.fig
%      ROBOTRRR, by itself, creates a new ROBOTRRR or raises the existing
%      singleton*.
%
%      H = ROBOTRRR returns the handle to a new ROBOTRRR or the handle to
%      the existing singleton*.
%
%      ROBOTRRR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTRRR.M with the given input arguments.
%
%      ROBOTRRR('Property','Value',...) creates a new ROBOTRRR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RobotRRR_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RobotRRR_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help RobotRRR
% Last Modified by GUIDE v2.5 17-Sep-2018 19:00:44
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RobotRRR_OpeningFcn, ...
                   'gui_OutputFcn',  @RobotRRR_OutputFcn, ...
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
% End initialization code - DO NOT EDIT
% --- Executes just before RobotRRR is made visible.
function RobotRRR_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RobotRRR (see VARARGIN)
% Choose default command line output for RobotRRR
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes RobotRRR wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% --- Outputs from this function are returned to the command line.
function varargout = RobotRRR_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
% --- Executes on button press in btn_Forward.
function btn_Forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('start_f')
%odczyt danych tet z gui oraz zamiana na wartosc w stopniach
Th_1 = str2double(handles.Theta_1.String)*pi/180;
Th_2 = str2double(handles.Theta_2.String)*pi/180;
Th_3 = str2double(handles.Theta_3.String)*pi/180;
global L_1 L_2 L_3
%dlugoosci ramion
L_1 = 20;  % [cm]
L_2 = 50;  % [cm]
L_3 = 40;  % [cm]
% przylaczenie kolejnych czlonow robota
L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
%stworzenie modelu robota
Robot = SerialLink(L);
Robot.name = 'RRR-Robot';  
% zmiene pomocnicze do 'stopniowego obrotu' oraz energi
global T_p1 T_p2 T_p3 Eccp Ecc;
% warunek do stworzenia zmienych pomocniczych w 1 urzyciu , 1 kroku petli
if exist('T_p1','var') == 0
T_p1=0;
T_p2=0;
T_p3=0;
Eccp=0;
Ecc=0;
end
% zmiene do rysowania lini
global h x; 
h = animatedline('color','r');
x = animatedline('color','g');      
T_p1;
T_p2;
T_p3;
Eccp;
Ecc;
% oznaczenia
global Th1_p Th2_p Th3_p Th1_b Th2_b Th3_b m1_r m2_r m3_r g v
v=0.1; % skok
Th1_p=0.1; %theta1'
Th2_p=0.1; %theta2'
Th3_p=0.1; %theta3'
Th1_b=0.01; %theta1''
Th2_b=0.01; %theta2''
Th3_b=0.01; %theta3''     
m1_r=24.3*pi/1000;    % [kg] masa ramienia 1
m2_r=60.75*pi/1000;   % [kg] masa ramienia 2
m3_r=48.6*pi/1000;    % [kg] masa ramienia 3
g=9.81; %przyspieszenie ziemksie [kg*m/s^2]
Tau_1_suma=[]; %wektory momentow
Tau_2_suma=[];
Tau_3_suma=[];
Xp=[];
Yp=[];
Zp=[];
%% 1 kat obrot
  if Th_1>T_p1
    while Th_1 > T_p1        
        T=Robot.fkine([T_p1 T_p2 T_p3]);
        Xp=[Xp,T.t(1)];
        Yp=[Yp,T.t(2)];
        Zp=[Zp,T.t(3)];
        s_1=sin(T_p1);s_2=sin(T_p2);s_3=sin(T_p3);c_1=cos(T_p1);c_2=cos(T_p2);c_3=cos(T_p3);s_23=sin(T_p2+T_p3);c_23=cos(T_p2+T_p3);
        Tau_1=abs((0.25*m2_r*L_2^2*c_2^2 + m3_r*L_2*L_3*c_2*c_23 + 0.25*m3_r*L_3^2*c_23^2 + m3_r*L_2^2*c_2^2)*Th1_b + (-0.5*m2_r*L_2^2*s_2*c_2 - 0.5*m3_r*L_3^2*s_23*c_23 - 2*m3_r*L_2^2*s_2*c_2 - m3_r*L_2*L_3*c_2*s_23 - m3_r*L_2*L_3*s_2*c_23)*Th1_p*Th2_b + (-0.5*m3_r*L_3^2*s_23*c_23 - m3_r*L_2*L_3*c_2*s_23)*Th1_p*Th3_b);
        Tau_1_suma=[Tau_1_suma Tau_1];
        addpoints(h,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_p1 T_p2 T_p3]);
                T_p1=T_p1+v;
    end  
else 
    while Th_1 < T_p1 -v       
        T=Robot.fkine([T_p1 T_p2 T_p3]);
        Xp=[Xp,T.t(1)];
        Yp=[Yp,T.t(2)];
        Zp=[Zp,T.t(3)];        
        s_1=sin(T_p1);s_2=sin(T_p2);s_3=sin(T_p3);c_1=cos(T_p1);c_2=cos(T_p2);c_3=cos(T_p3);s_23=sin(T_p2+T_p3);c_23=cos(T_p2+T_p3);
        Tau_1=abs((0.25*m2_r*L_2^2*c_2^2 + m3_r*L_2*L_3*c_2*c_23 + 0.25*m3_r*L_3^2*c_23^2 + m3_r*L_2^2*c_2^2)*Th1_b + (-0.5*m2_r*L_2^2*s_2*c_2 - 0.5*m3_r*L_3^2*s_23*c_23 - 2*m3_r*L_2^2*s_2*c_2 - m3_r*L_2*L_3*c_2*s_23 - m3_r*L_2*L_3*s_2*c_23)*Th1_p*Th2_b + (-0.5*m3_r*L_3^2*s_23*c_23 - m3_r*L_2*L_3*c_2*s_23)*Th1_p*Th3_b);
        Tau_1_suma=[Tau_1_suma Tau_1];
        addpoints(h,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_p1 T_p2 T_p3]);
                T_p1=T_p1-v;
    end
  end
%% 2 kat obrot
    if Th_2>T_p2
    while Th_2 > T_p2
        T=Robot.fkine([T_p1 T_p2 T_p3]);
        Xp=[Xp,T.t(1)];
        Yp=[Yp,T.t(2)];
        Zp=[Zp,T.t(3)]; 
        s_1=sin(T_p1);s_2=sin(T_p2);s_3=sin(T_p3);c_1=cos(T_p1);c_2=cos(T_p2);c_3=cos(T_p3);s_23=sin(T_p2+T_p3);c_23=cos(T_p2+T_p3);
        Tau_2=abs((0.25*m2_r*L_2^2 + m3_r*L_2^2 + m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th2_b + (0.5*m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th3_b + (0.25*m2_r*L_2^2*s_2*c_2 + m3_r*L_2^2*s_2*c_2 + 0.5*m3_r*L_2*L_3*s_2*c_23 + 0.5* m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 - m3_r*L_2*L_3*s_3*Th2_p*Th3_p - 0.5*m3_r*L_2*L_3*s_3*Th3_p^2 + m3_r*L_2*c_2*g + 0.5*m3_r*L_3*c_23*g + 0.5*m2_r*L_2*c_2*g);
        Tau_2_suma=[Tau_2_suma Tau_2];
        addpoints(h,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_p1 T_p2 T_p3]);
                T_p2=T_p2+v;
    end 
else 
    while Th_2 < T_p2 -v
        T=Robot.fkine([T_p1 T_p2 T_p3]);
        Xp=[Xp,T.t(1)];
        Yp=[Yp,T.t(2)];
        Zp=[Zp,T.t(3)];
        s_1=sin(T_p1);s_2=sin(T_p2);s_3=sin(T_p3);c_1=cos(T_p1);c_2=cos(T_p2);c_3=cos(T_p3);s_23=sin(T_p2+T_p3);c_23=cos(T_p2+T_p3);
        Tau_2=abs((0.25*m2_r*L_2^2 + m3_r*L_2^2 + m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th2_b + (0.5*m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th3_b + (0.25*m2_r*L_2^2*s_2*c_2 + m3_r*L_2^2*s_2*c_2 + 0.5*m3_r*L_2*L_3*s_2*c_23 + 0.5* m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 - m3_r*L_2*L_3*s_3*Th2_p*Th3_p - 0.5*m3_r*L_2*L_3*s_3*Th2_p^3 + m3_r*L_2*c_2*g + 0.5*m3_r*L_3*c_23*g + 0.5*m2_r*L_2*c_2*g);
        Tau_2_suma=[Tau_2_suma Tau_2]
        addpoints(h,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_p1 T_p2 T_p3]);
                T_p2=T_p2-v;
    end
    end
%% kat 3 obrot
      if Th_3>T_p3
    while Th_3 > T_p3
        T=Robot.fkine([T_p1 T_p2 T_p3]);
        Xp=[Xp,T.t(1)];
        Yp=[Yp,T.t(2)];
        Zp=[Zp,T.t(3)];
        s_1=sin(T_p1);s_2=sin(T_p2);s_3=sin(T_p3);c_1=cos(T_p1);c_2=cos(T_p2);c_3=cos(T_p3);s_23=sin(T_p2+T_p3);c_23=cos(T_p2+T_p3);
        Tau_3=abs((0.25*m3_r*L_3^2 + 0.5*m3_r*L_2*L_3*c_3)*Th2_b + 0.25*m3_r*L_3^2*Th3_b + (0.5*m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 + 0.5*m3_r*L_2*L_3*s_3*Th2_p^2 + 0.5*m3_r*L_3*c_23*g);
        Tau_3_suma=[Tau_3_suma Tau_3];
        addpoints(h,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_p1 T_p2 T_p3]);
                T_p3=T_p3+v;
    end 
else 
    while Th_3 < T_p3 -v
        T=Robot.fkine([T_p1 T_p2 T_p3]);
        Xp=[Xp,T.t(1)];
        Yp=[Yp,T.t(2)];
        Zp=[Zp,T.t(3)];
        s_1=sin(T_p1);s_2=sin(T_p2);s_3=sin(T_p3);c_1=cos(T_p1);c_2=cos(T_p2);c_3=cos(T_p3);s_23=sin(T_p2+T_p3);c_23=cos(T_p2+T_p3);
        Tau_3=abs((0.25*m3_r*L_3^2 + 0.5*m3_r*L_2*L_3*c_3)*Th2_b + 0.25*m3_r*L_3^2*Th3_b + (0.5*m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 + 0.5*m3_r*L_2*L_3*s_3*Th2_p^2 + 0.5*m3_r*L_3*c_23*g);
        Tau_3_suma=[Tau_3_suma Tau_3];
        addpoints(h,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_p1 T_p2 T_p3]);
                T_p3=T_p3-v;
    end
      end
Tau_1_wynik=sum(Tau_1_suma)  
Tau_2_wynik=sum(Tau_2_suma)  
Tau_3_wynik=sum(Tau_3_suma) 
  Robot.plot([Th_1 Th_2 Th_3]);
  legend('Forward','Inverse')
%oblicza polozenia x,y,z za pomoca kinematyki w porzod
T = Robot.fkine([Th_1 Th_2 Th_3]);
addpoints(h,T.t(1),T.t(2),T.t(3))
%przekazanie pozycji do gui 
handles.Pos_X.String = num2str(floor(T.t(1)));
handles.Pos_Y.String = num2str(floor(T.t(2)));
handles.Pos_Z.String = num2str(floor(T.t(3)));
T_p1=Th_1;
T_p2=Th_2;
T_p3=Th_3;
%obliczenie energi ca³kowitej
Ec=Tau_1_wynik*Th1_p+Tau_2_wynik*Th2_p+Tau_3_wynik*Th3_p
Ecc=Eccp+Ec;
%przekazanie wartosci do gui 
handles.Energy.String = num2str(Ecc);
%zmiena pomocnicza
Eccp=Ecc;
disp('koniec_f')
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function btn_Inverse_Callback(hObject, eventdata, handles)
disp('start_i')
%odczyt aktaulnego po³o¿enia z gui
PX = str2double(handles.Pos_X.String);
PY = str2double(handles.Pos_Y.String);
PZ = str2double(handles.Pos_Z.String);
% zmiene ramion i nergi
global L_1 L_2 L_3 Eccp Ecc
%dlugoosci ramion
L_1 = 20;  % [cm]
L_2 = 50;  % [cm]
L_3 = 40;  % [cm]
global Th1_p Th2_p Th3_p Th1_b Th2_b Th3_b m1_r m2_r m3_r g v
v=0.1; % skok
Th1_p=0.1; %theta1'
Th2_p=0.1; %theta2'
Th3_p=0.1; %theta3'
Th1_b=0.01; %theta1''
Th2_b=0.01; %theta2''
Th3_b=0.01; %theta3''     
m1_r=24.3*pi/1000;    % [kg] masa ramienia 1
m2_r=60.75*pi/1000;   % [kg] masa ramienia 2
m3_r=48.6*pi/1000;    % [kg] masa ramienia 3
g=9.81; %przyspieszenie ziemksie [kg*m/s^2]
%przy³aczanie kolejnych cz³onów robota
L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
%budowa modelu robota
Robot = SerialLink(L);
Robot.name = 'RRR-Robot';
%macierz po³o¿eñ
T = [1 0 0 PX;
     0 1 0 PY;
     0 0 1 PZ;
     0 0 0 1];
%odwrotne zadanie kinematyki
 J = Robot.ikine(T,'q0',[0,0,0],'mask',[1 1 1 0 0 0]);
 %zmienie tet
 J1=round(J(1)*180/pi,1);
 J2=round(J(2)*180/pi,1);
 J3=round(J(3)*180/pi,1);
 %przekazanie wartosci k¹tów teta do gui
 handles.Theta_1.String = num2str(round((J1),1));
 handles.Theta_2.String = num2str(round((J2),1));
 handles.Theta_3.String = num2str(round((J3),1));
Th1= J(1);
Th2= J(2);
Th3= J(3);
%zmiene pomocnicze do obrtów
global T_pi1 T_pi2 T_pi3;
global T_p1 T_p2 T_p3; 
if exist('T_pi1','var') == 0
T_pi1=0;
T_pi2=0;
T_pi3=0;
end
T_pi1=T_p1;
T_pi2=T_p2;
T_pi3=T_p3;
%zmiene do zaznacznia drogi
global h x; 
x = animatedline('color','g');
%zmiene pomocnicze
Tau_1_suma=[];
Tau_2_suma=[];
Tau_3_suma=[];
Xo=[];
Yo=[];
Zo=[];
%% 1 kat obrot
  if Th1>T_pi1
    while Th1-T_pi1>v
        T=Robot.fkine([T_pi1 T_pi2 T_pi3]);
        Xo=[Xo,T.t(1)];
        Yo=[Yo,T.t(2)];
        Zo=[Zo,T.t(3)];
        s_1=sin(T_pi1);s_2=sin(T_pi2);s_3=sin(T_pi3);c_1=cos(T_pi1);c_2=cos(T_pi2);c_3=cos(T_pi3);s_23=sin(T_pi2+T_pi3);c_23=cos(T_pi2+T_pi3);
        Tau_1=abs((0.25*m2_r*L_2^2*c_2^2 + m3_r*L_2*L_3*c_2*c_23 + 0.25*m3_r*L_3^2*c_23^2 + m3_r*L_2^2*c_2^2)*Th1_b + (-0.5*m2_r*L_2^2*s_2*c_2 - 0.5*m3_r*L_3^2*s_23*c_23 - 2*m3_r*L_2^2*s_2*c_2 - m3_r*L_2*L_3*c_2*s_23 - m3_r*L_2*L_3*s_2*c_23)*Th1_p*Th2_b + (-0.5*m3_r*L_3^2*s_23*c_23 - m3_r*L_2*L_3*c_2*s_23)*Th1_p*Th3_b);
        Tau_1_suma=[Tau_1_suma Tau_1];
        addpoints(x,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_pi1 T_pi2 T_pi3]);
                T_pi1=T_pi1+v;
    end  
else 
    while abs(Th1 - T_pi1) > v
        T=Robot.fkine([T_pi1 T_pi2 T_pi3]);
        Xo=[Xo,T.t(1)];
        Yo=[Yo,T.t(2)];
        Zo=[Zo,T.t(3)];
        s_1=sin(T_pi1);s_2=sin(T_pi2);s_3=sin(T_pi3);c_1=cos(T_pi1);c_2=cos(T_pi2);c_3=cos(T_pi3);s_23=sin(T_pi2+T_pi3);c_23=cos(T_pi2+T_pi3);
        Tau_1=abs((0.25*m2_r*L_2^2*c_2^2 + m3_r*L_2*L_3*c_2*c_23 + 0.25*m3_r*L_3^2*c_23^2 + m3_r*L_2^2*c_2^2)*Th1_b + (-0.5*m2_r*L_2^2*s_2*c_2 - 0.5*m3_r*L_3^2*s_23*c_23 - 2*m3_r*L_2^2*s_2*c_2 - m3_r*L_2*L_3*c_2*s_23 - m3_r*L_2*L_3*s_2*c_23)*Th1_p*Th2_b + (-0.5*m3_r*L_3^2*s_23*c_23 - m3_r*L_2*L_3*c_2*s_23)*Th1_p*Th3_b);
        Tau_1_suma=[Tau_1_suma Tau_1];
        addpoints(x,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_pi1 T_pi2 T_pi3]);
                T_pi1=T_pi1-v;
    end
  end
%% 2 kat obrot
  if Th2>T_pi2
    while Th2-T_pi2>v    
        T=Robot.fkine([T_pi1 T_pi2 T_pi3]);
        Xo=[Xo,T.t(1)];
        Yo=[Yo,T.t(2)];
        Zo=[Zo,T.t(3)];
        s_1=sin(T_pi1);s_2=sin(T_pi2);s_3=sin(T_pi3);c_1=cos(T_pi1);c_2=cos(T_pi2);c_3=cos(T_pi3);s_23=sin(T_pi2+T_pi3);c_23=cos(T_pi2+T_pi3);
        Tau_2=abs((0.25*m2_r*L_2^2 + m3_r*L_2^2 + m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th2_b + (0.5*m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th3_b + (0.25*m2_r*L_2^2*s_2*c_2 + m3_r*L_2^2*s_2*c_2 + 0.5*m3_r*L_2*L_3*s_2*c_23 + 0.5* m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 - m3_r*L_2*L_3*s_3*Th2_p*Th3_p - 0.5*m3_r*L_2*L_3*s_3*Th2_p^3 + m3_r*L_2*c_2*g + 0.5*m3_r*L_3*c_23*g + 0.5*m2_r*L_2*c_2*g);
        Tau_2_suma=[Tau_2_suma Tau_2]
        addpoints(x,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_pi1 T_pi2 T_pi3]);
                T_pi2=T_pi2+v;
    end  
else 
    while abs(Th2 - T_pi2) > v
        T=Robot.fkine([T_pi1 T_pi2 T_pi3]);
        Xo=[Xo,T.t(1)];
        Yo=[Yo,T.t(2)];
        Zo=[Zo,T.t(3)];
        s_1=sin(T_pi1);s_2=sin(T_pi2);s_3=sin(T_pi3);c_1=cos(T_pi1);c_2=cos(T_pi2);c_3=cos(T_pi3);s_23=sin(T_pi2+T_pi3);c_23=cos(T_pi2+T_pi3);
        Tau_2=abs((0.25*m2_r*L_2^2 + m3_r*L_2^2 + m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th2_b + (0.5*m3_r*L_2*L_3*c_3 + 0.25*m3_r*L_3^2)*Th3_b + (0.25*m2_r*L_2^2*s_2*c_2 + m3_r*L_2^2*s_2*c_2 + 0.5*m3_r*L_2*L_3*s_2*c_23 + 0.5* m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 - m3_r*L_2*L_3*s_3*Th2_p*Th3_p - 0.5*m3_r*L_2*L_3*s_3*Th2_p^3 + m3_r*L_2*c_2*g + 0.5*m3_r*L_3*c_23*g + 0.5*m2_r*L_2*c_2*g);
        Tau_2_suma=[Tau_2_suma Tau_2]
        addpoints(x,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_pi1 T_pi2 T_pi3]);
                T_pi2=T_pi2-v;
    end
  end
%% 3 kat obrot
  if Th3>T_pi3
    while Th3-T_pi3>v
        T=Robot.fkine([T_pi1 T_pi2 T_pi3]);
        Xo=[Xo,T.t(1)];
        Yo=[Yo,T.t(2)];
        Zo=[Zo,T.t(3)];
        s_1=sin(T_pi1);s_2=sin(T_pi2);s_3=sin(T_pi3);c_1=cos(T_pi1);c_2=cos(T_pi2);c_3=cos(T_pi3);s_23=sin(T_pi2+T_pi3);c_23=cos(T_pi2+T_pi3);
        Tau_3=abs((0.25*m3_r*L_3^2 + 0.5*m3_r*L_2*L_3*c_3)*Th2_b + 0.25*m3_r*L_3^2*Th3_b + (0.5*m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 + 0.5*m3_r*L_2*L_3*s_3*Th2_p^2 + 0.5*m3_r*L_3*c_23*g);
        Tau_3_suma=[Tau_3_suma Tau_3]
        addpoints(x,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_pi1 T_pi2 T_pi3]);
                T_pi3=T_pi3+v;
    end  
else 
    while abs(Th3 - T_pi3) > v
        T=Robot.fkine([T_pi1 T_pi2 T_pi3]);
        Xo=[Xo,T.t(1)];
        Yo=[Yo,T.t(2)];
        Zo=[Zo,T.t(3)];
        s_1=sin(T_pi1);s_2=sin(T_pi2);s_3=sin(T_pi3);c_1=cos(T_pi1);c_2=cos(T_pi2);c_3=cos(T_pi3);s_23=sin(T_pi2+T_pi3);c_23=cos(T_pi2+T_pi3);
        Tau_3=abs((0.25*m3_r*L_3^2 + 0.5*m3_r*L_2*L_3*c_3)*Th2_b + 0.25*m3_r*L_3^2*Th3_b + (0.5*m3_r*L_2*L_3*c_2*s_23 + 0.25*m3_r*L_3^2*s_23*c_23)*Th1_p^2 + 0.5*m3_r*L_2*L_3*s_3*Th2_p^2 + 0.5*m3_r*L_3*c_23*g);
        Tau_3_suma=[Tau_3_suma Tau_3]
        addpoints(x,T.t(1),T.t(2),T.t(3))
        Robot.plot([T_pi1 T_pi2 T_pi3]);
        T_pi3=T_pi3-v;
    end
  end
T_pi1=Th1;
T_pi2=Th2;
T_pi3=Th3;
T_p1=T_pi1;
T_p2=T_pi2;
T_p3=T_pi3;
Tau_1_wynik=sum(Tau_1_suma)  
Tau_2_wynik=sum(Tau_2_suma)
Tau_3_wynik=sum(Tau_3_suma)
%oblicznie zu¿ycia energii
Ec=Tau_1_wynik*Th1_p+Tau_2_wynik*Th2_p+Tau_3_wynik*Th3_p
Ecc=Eccp+Ec;
%przekazanie zmienej do gui
handles.Energy.String = num2str(Ecc);
Eccp=Ecc;
Robot.plot([Th1 Th2 Th3])
T=Robot.fkine([Th1 Th2 Th3]);
addpoints(x,T.t(1),T.t(2),T.t(3))
disp('koniec_i')
function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double
% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double
% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double
% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double
% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double
% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double
% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Speed_Callback(hObject, eventdata, handles)
% hObject    handle to Speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of Speed as text
%        str2double(get(hObject,'String')) returns contents of Speed as a double
% --- Executes during object creation, after setting all properties.
function Speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Energy_Callback(hObject, eventdata, handles)
% hObject    handle to Energy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Energy as text
%        str2double(get(hObject,'String')) returns contents of Energy as a double
% --- Executes during object creation, after setting all properties.
function Energy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Energy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in btn_Clearenergyconsumption.
function btn_Clearenergyconsumption_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Clearenergyconsumption (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ecc Eccp
Ecc=0;
Eccp=0;
handles.Energy.String = num2str(Ecc);
% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global h x;
 clearpoints(h)
 clearpoints(x)
% --- Executes on button press in ST_EC.
function ST_EC_Callback(hObject, eventdata, handles)
% hObject    handle to ST_EC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function v_Callback(hObject, eventdata, handles)
% hObject    handle to Speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Speed as text
%        str2double(get(hObject,'String')) returns contents of Speed as a double
% --- Executes during object creation, after setting all properties.
function v_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes on button press in btn_changespeed.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to btn_changespeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes during object deletion, before destroying properties.
function Speed_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to Speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes during object deletion, before destroying properties.
function pushbutton9_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
