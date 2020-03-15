clc 
clear
joint0=180/pi*load('LineTrajactory0.txt');
joint1=180/pi*load('LineTrajactory.txt');

joint=[joint0;joint1];
joint=180/pi*load('LineTrajactory.txt');
k=1:length(joint(:,1));
k=k';
t=0.1*k;
jt=[k,t,joint];