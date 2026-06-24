% Parametri robot
run('/Users/giadatangredi/Documents/MATLAB/Examples/R2023b/control_deeplearning/TrainBipedRobotToWalkUsingReinforcementLearningAgentsExample/robotParametersRL.m')

% Crea ambiente senza agent caricato
env = rlSimulinkEnv('rlWalkingBipedRobot','rlWalkingBipedRobot/RL Agent');

% Ricava info
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
numObs = obsInfo.Dimension(1);
numAct = actInfo.Dimension(1);

% Crea network Critic
statePath = featureInputLayer(numObs,'Normalization','none','Name','state');
actionPath = featureInputLayer(numAct,'Normalization','none','Name','action');
commonPath = concatenationLayer(1,2,'Name','concat');
fc1 = fullyConnectedLayer(24,'Name','fc1');
relu1 = reluLayer('Name','relu1');
criticOut = fullyConnectedLayer(1,'Name','CriticOutput');

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = addLayers(criticNetwork,fc1);
criticNetwork = addLayers(criticNetwork,relu1);
criticNetwork = addLayers(criticNetwork,criticOut);

criticNetwork = connectLayers(criticNetwork,'state','concat/in1');
criticNetwork = connectLayers(criticNetwork,'action','concat/in2');
criticNetwork = connectLayers(criticNetwork,'concat','fc1');
criticNetwork = connectLayers(criticNetwork,'fc1','relu1');
criticNetwork = connectLayers(criticNetwork,'relu1','CriticOutput');

criticOpts = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1);
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'state'},'Action',{'action'},criticOpts);

% Crea network Actor
actorLayers = [
    featureInputLayer(numObs,'Normalization','none','Name','state')
    fullyConnectedLayer(24,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(numAct,'Name','fc2')
    tanhLayer('Name','tanh1')
    ];
actorNetwork = layerGraph(actorLayers);
actorOpts = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1);
actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'state'},'Action',{'tanh1'},actorOpts);

% Opzioni agente
agentOpts = rlDDPGAgentOptions(...
    'SampleTime', 0.005, ...
    'TargetSmoothFactor', 1e-3, ...
    'ExperienceBufferLength', 1e5, ...
    'DiscountFactor', 0.99, ...
    'MiniBatchSize', 64);

% Crea agente non addestrato
agent = rlDDPGAgent(actor,critic,agentOpts);

% Opzioni simulazione
simOptions = rlSimulationOptions('MaxSteps',10000);

% Simula
experience = sim(env,agent,simOptions);