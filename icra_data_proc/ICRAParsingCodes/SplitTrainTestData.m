% Input: Randomly split data.
% V, F: N*3.
% ratio_train: splitting ration for training data.
function [V_train, V_test, F_train, F_test] = SplitTrainTestData(V, F, ratio_train)
NData = size(V, 1);
NDataTrain = ceil(NData * ratio_train);
index_perm = randperm(NData);

V_train = V(index_perm(1:NDataTrain), :);
V_test = V(index_perm(NDataTrain+1:end), :);

F_train = F(index_perm(1:NDataTrain), :);
F_test = F(index_perm(NDataTrain+1:end), :);

end

