function agent = createDDPGAgentParam2(numObs, obsInfo, numAct, actInfo, Ts)
% Walking Robot -- DDPG Agent Setup Script
% Copyright 2020 The MathWorks, Inc.

%% Create the actor and critic networks using the createNetworks helper function
[criticNetwork,~,actorNetwork] = createNetworks(numObs,numAct);

%% Specify options for the critic and actor representations using rlOptimizerOptions
criticOptions = rlOptimizerOptions('Optimizer','adam','LearnRate',3e-4,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
actorOptions = rlOptimizerOptions('Optimizer','adam','LearnRate',3e-4,...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);

%% Create critic and actor representations using specified networks and
% options
critic = rlQValueFunction(criticNetwork,obsInfo,actInfo,'ObservationInputNames','observation','ActionInputNames','action');
actor  = rlContinuousDeterministicActor(actorNetwork,obsInfo,actInfo);

%% Specify DDPG agent options
agentOptions = rlDDPGAgentOptions;
agentOptions.SampleTime = Ts;
agentOptions.DiscountFactor = 0.98;
agentOptions.MiniBatchSize = 512;
agentOptions.ExperienceBufferLength = 1e6;
agentOptions.TargetSmoothFactor = 0.008;
agentOptions.NoiseOptions.MeanAttractionConstant = 0.8;
agentOptions.NoiseOptions.Variance = 0.3;
agentOptions.ActorOptimizerOptions = actorOptions;
agentOptions.CriticOptimizerOptions = criticOptions;

%% Create agent using specified actor representation, critic representation and agent options
agent = rlDDPGAgent(actor,critic,agentOptions);