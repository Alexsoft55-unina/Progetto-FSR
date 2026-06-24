robotParametersRL;
previousRngState = rng(0, 'twister');
mdl = 'rlWalkingBipedRobot';
open_system(mdl);


numObs=29;
obsInfo=rlNumericSpec([numObs 1]);
obsInfo.Name= "observations";

numAct= 6;
actInfo=rlNumericSpec([numAct 1], LowerLimit=-1, UpperLimit=1);
actInfo.Name= "foot_torque";

blk=mdl + "/RL Agent";
env=rlSimulinkEnv(mdl, blk, obsInfo, actInfo);
env.ResetFcn= @(in) walkerResetFcn(in, ...
    upper_leg_length/100, ...
    lower_leg_length/100, ...
    h/100);
Ts=0.02;
Tf=1.5;
agent=createDDPGAgentParam2(numObs, obsInfo, numAct, actInfo, Ts);
rng(0, "twister");
actor = getActor(agent);
critic = getCritic(agent);
actorNet = getModel(actor);
criticNet = getModel(critic(1));


maxEpisodes = 1000;
maxSteps = floor(Tf/Ts);
trainOpts = rlTrainingOptions(...
    MaxEpisodes=maxEpisodes,...
    MaxStepsPerEpisode=maxSteps,...
    ScoreAveragingWindowLength=250,...
    StopTrainingCriteria="none");


% Avvia addestramento
trainingStats_2 = train(agent, env, trainOpts);

% Salva agente e statistiche addestramento
save('trainingDDPGParam2.mat', 'agent', 'trainingStats_2');

% ------- Visualizza grafico della ricompensa cumulativa per episodio --------
figure
plot(trainingStats_2.EpisodeIndex, trainingStats_2.EpisodeReward)
xlabel('Episodi')
ylabel('Ricompensa cumulativa')
title('Andamento ricompensa cumulativa durante l''addestramento')
grid on