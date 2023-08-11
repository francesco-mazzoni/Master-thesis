clc
clear all
close all

%%

syms x y z psi theta

%%

M1 = [1,0,0,x; 0,1,0,y; 0,0,1,z; 0,0,0,1]

%%

M2 = [cos(psi), -sin(psi), 0, 0;sin(psi), cos(psi), 0, 0;...
    0, 0, 1, 0; 0, 0, 0, 1]

%%

M3 = [0,0,1,0;0,1,0,0;-1,0,0,0;0,0,0,1]

%%

M4 = [0,1,0,0; -1,0,0,0;0,0,1,0;0,0,0,1]

%%

M5 = [1,0,0,0;0,cos(theta),sin(theta),0;0,-sin(theta),cos(theta),0;0,0,0,1]

%%

CW = inv(M1*M2*M3*M4*M5)

%%

syms a b

%%

CW(3,:)*[a;b;0;1]

%%

WC = M1*M2*M3*M4*M5

%%

syms u v w

%%

WC*[0;0;0;1]+WC*[u*w;v*w;w;0]