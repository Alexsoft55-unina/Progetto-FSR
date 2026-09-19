% =========================================================================
% Script di Inizializzazione — Modello Lagrangiano Robot Bipede "Scooter"
% Convenzione ESATTA del paper (Cui et al., 2022, Table 1):
%   qw = rotazione ruota
%   q1 = angolo GINOCCHIO (attuato)
%   q2 = angolo ANCA (attuato)
%   q3 = angolo TORSO, assoluto rispetto alla verticale (NON attuato,
%        evolve per dinamica libera)
%   q0 = angolo CAVIGLIA (NON attuato, vincolo algebrico: q0 = q1 - q2 + q3)
%
% q1=q2=q3=0  ->  robot perfettamente dritto in piedi, tutti i link verticali.
% =========================================================================
clc; clear;

%% 1. Parametri fisici
robot_params.mw = 3.5;
robot_params.Iw = 0.1;
robot_params.meq = 60;
robot_params.Rw = 0.127;
robot_params.leq = 1 ;     % altezza torso
robot_params.g = 9.81;

robot_params.bw = .01;


