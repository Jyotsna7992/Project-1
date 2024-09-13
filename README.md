clc;
clear;

%% Parameters
numNodes = 100; % Number of nodes
fieldDim = 100; % Field dimensions (100x100 sqm)
numAnts = 50; % Number of ants in ACO
numIterations = 100; % Number of iterations in ACO
packetSize = 4000; % Packet size in bits


% Energy model parameters
Eelec = 50e-9; % Energy to run the radio (J/bit)
Eamp = 100e-12; % Energy for amplification (J/bit/m^2)
initialEnergy = 0.5; % Initial energy for each node (J)

% Pheromone and heuristic parameters for ACO
alpha = 1; % Pheromone importance
beta = 2; % Heuristic importance
evaporationRate = 0.5; % Pheromone evaporation rate

% Sink node position
sink = [50, 50]; % Sink placed in the center of the field

%% Generate random positions for nodes
nodes = rand(numNodes, 2) * fieldDim;

% Introduce unbalanced node distribution (40% in a smaller area)
unbalancedNodes = round(numNodes*0.4);
nodes(1:unbalancedNodes, :) = rand(unbalancedNodes, 2) * 25; % Cluster in [0,25]x[0,25] region

% Initialize energy for each node
nodeEnergy = initialEnergy * ones(numNodes, 1);

%% Plot node deployment
figure;
scatter(nodes(:,1), nodes(:,2), 'ro'); hold on;
scatter(sink(1), sink(2), 100, 'bs', 'filled'); % Sink in blue
title('WSN Node Deployment');
xlabel('m');
ylabel('m');
grid on;

%% ACO for Clustering

% Distance matrix between nodes
distance = zeros(numNodes, numNodes);
for i = 1:numNodes
    for j = i+1:numNodes
        distance(i,j) = sqrt((nodes(i,1) - nodes(j,1))^2 + (nodes(i,2) - nodes(j,2))^2);
        distance(j,i) = distance(i,j); % Symmetric distance matrix
    end
end

% Initialize pheromones
pheromone = ones(numNodes, numNodes); % Initial pheromone levels
distance = rand(numNodes, numNodes); % Random distance matrix as an example

% Run ACO for cluster formation
for iter = 1:numIterations
    antPaths = zeros(numAnts, numNodes); % Store the path for each ant
    for ant = 1:numAnts
        currentNode = randi([1 numNodes]); % Start from random node
        visited = false(1, numNodes); % Track visited nodes
        visited(currentNode) = true;
        path = currentNode;
        
        for step = 2:numNodes
    % Calculate transition probabilities based on pheromone and distance
    pheromoneVisible = pheromone(currentNode, :) .^ alpha;
    heuristic = (1./distance(currentNode, :)) .^ beta;
    
    % Ensure the heuristic is finite and valid (handling divide-by-zero or Inf)
    heuristic(~isfinite(heuristic)) = 0;
    
    % Calculate the probabilities of moving to the next node
    probabilities = pheromoneVisible .* heuristic .* (~visited);
    
    % Check if all probabilities are zero (e.g., due to very small pheromone or distance)
    if sum(probabilities) == 0
        % If all probabilities are zero, choose a random unvisited node
        unvisitedNodes = find(~visited);
        nextNode = unvisitedNodes(randi(length(unvisitedNodes)));
    else
        % Normalize the probabilities
        probabilities = probabilities / sum(probabilities);
        
        % Select the next node based on the calculated probabilities
        nextNode = randsample(1:numNodes, 1, true, probabilities);
    end
    
    % Update the path and mark the node as visited
    path = [path, nextNode];
    visited(nextNode) = true;
    currentNode = nextNode;
end


antPaths(ant, :) = path;
    
    % Update pheromones based on paths
    
    pheromone = (1 - evaporationRate) * pheromone;
    for ant = 1:numAnts
        path = antPaths(ant,:);
        for step = 1:(numNodes-1)
            i = max(1, min(numNodes, path(step)));
            j = max(1, min(numNodes, path(step+1)));
        
            % Ensure no division by zero
              dist = distance(i, j);
              if dist == 0
                 dist = eps; % Handle zero distance
              end
        
        % Update pheromone levels
        pheromone(i, j) = pheromone(i, j) + 1/dist;
    end
  end
 end
end

%% Identify Cluster Heads (CHs)
% For simplicity, randomly select CHs (use ACO result in a refined version)
numCHs = round(numNodes/10); % Approximate number of cluster heads
CHs = unique(randi([1 numNodes], 1, numCHs)); % Random selection of CHs

% Plot Cluster Heads
scatter(nodes(CHs, 1), nodes(CHs, 2), 100, 'g*', 'filled'); % CHs marked in green

%% Shortest Path Calculation from Nodes to CHs and Sink
sinkDistances = zeros(length(CHs), 1); % Distance between CHs and sink
for i = 1:length(CHs)
    sinkDistances(i) = sqrt((nodes(CHs(i),1) - sink(1))^2 + (nodes(CHs(i),2) - sink(2))^2);
end

memberCHDistances = zeros(numNodes, 1); % Distance between nodes and CHs
for i = 1:numNodes
    [~, nearestCH] = min(sqrt(sum((nodes(CHs,:) - nodes(i,:)).^2, 2)));
    memberCHDistances(i) = sqrt(sum((nodes(CHs(nearestCH),:) - nodes(i,:)).^2));
end

% Plot shortest paths between nodes and CHs
for i = 1:numNodes
    [~, nearestCH] = min(sqrt(sum((nodes(CHs,:) - nodes(i,:)).^2, 2)));
    plot([nodes(i,1), nodes(CHs(nearestCH),1)], [nodes(i,2), nodes(CHs(nearestCH),2)], 'g--');
end

% Plot shortest paths between CHs and sink
for i = 1:length(CHs)
    plot([nodes(CHs(i),1), sink(1)], [nodes(CHs(i),2), sink(2)], 'b-');
end

%% Energy Consumption per Transmission
energyConsumption = zeros(numNodes, 1);
for i = 1:numNodes
    d = memberCHDistances(i); % Distance from node to CH
    energyConsumption(i) = (Eelec + Eamp * d^2) * packetSize; % Energy per transmission
end

%% Simulation over Multiple Rounds
numRounds = 4000;
aliveNodes = zeros(numRounds, 1); % Track number of alive nodes
throughput = zeros(numRounds, 1); % Track throughput
delay = zeros(numRounds, 1); % Track delay

for round = 1:numRounds
    % Update node energy based on energy consumption
    nodeEnergy = nodeEnergy - energyConsumption;
    aliveNodes(round) = sum(nodeEnergy > 0); % Count of alive nodes
    
    % Simplified throughput (alive nodes * packet size)
    throughput(round) = aliveNodes(round) * packetSize;
    
    % Simplified delay (based on distance from nodes to CH)
    delay(round) = mean(memberCHDistances) / 1e6; % Delay in milliseconds
end

%% Plot Results

% Plot No. of Rounds vs No. of Alive Nodes
figure(3);
plot(1:numRounds, aliveNodes, '-');
title('No. of Rounds vs Alive Nodes');
xlabel('Rounds');
ylabel('Alive Nodes');

% Plot No. of Rounds vs Throughput
figure(4);
plot(1:numRounds, throughput, '-');
title('No. of Rounds vs Throughput (bps)');
xlabel('Rounds');
ylabel('Throughput (bps)');

% Plot No. of Rounds vs Delay
figure(5);
plot(1:numRounds, delay, '-.');
title('No. of Rounds vs Delay (ms)');
xlabel('Rounds');
ylabel('Delay (ms)');
