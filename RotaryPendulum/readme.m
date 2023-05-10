%% readme.html
% Copyright 2015 The MathWorks, Inc.

%% Bundle Contents
% This bundle contains software models and additional support materials and
% is organized in three folders:
%
% * |media| : contains images and the hardware construction manual
% * |models| : contains all the S/W models
% * |utilities| : includes the startup script (for easy access to all
% models) and the parameter file


%% Demo Setup and Execution
% Before you run any of the demos make sure to add the bundle folder and
% all its subfolders to the MATLAB search path.
% 
% Execute the startup script in the |utilities| folder to open up the html
% menu that navigates you through all the demo examples.

%% Description
% This bundle demonstrates two different modeling approaches for physical 
% systems: a traditional approach using textual programming (with MATLAB 
% and Simulink) and through MathWorks' multi-body simulation environment, 
% SimMechanics. As a use case for teaching in academia the two modeling 
% approaches are applied for the controlled (inverted), rotary pendulum 
% system. The system is visualized and animated using Simulink 3D Animation
% with a simple virtual reality world and a more sophisticated, imported 
% CAD model on the one hand, and with the SimMechanics-built model on the 
% other hand.
%
% Also included in this bundle are the corresponding hardware implementation 
% models for LEGO Mindstorms NXT and EV3, a video showing the controlled
% LEGO system in action, and a hardware construction manual.
%
% For the traditional modeling approach the Euler-Lagrange tool is used to 
% derive the system differential equations: 
% <http://www.mathworks.com/matlabcentral/fileexchange/49796-euler-lagrange-tool-package>
%
% Note: these are the demos files that are used in the MATLAB EXPO Germany 2015 Masterclass on modeling physical system in academia (http://www.matlabexpo.com/de/2015/modellierung-physikalischer-systeme-in-der-lehre-technologische-ansatze-und-deren-didaktik.html).

%% Required Products
%% 
% * MATLAB
% * Simulink
% * Simscape
% * SimMechanics
% * Stateflow

%% Release Information
% Tested on MATLAB 8.5 (R2015a)

%% Other Requirements
% Deploying the models to hardware requires the corresponding, complementary
% hardware support package for LEGO NXT or EV3 to be installed. Simulink 3D 
% Animation, and the Aerospace blockset are recommended to run the demos 
% with all the bells and whistles.