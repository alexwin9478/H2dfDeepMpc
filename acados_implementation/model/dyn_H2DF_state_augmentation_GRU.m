function GRU_states = dyn_H2DF_state_augmentation_GRU(last_LSTM_states, input, Par)

% get last cell and last hidden states:
hiddenState = last_LSTM_states(1:Par.nHiddenStates);
%hiddenState = state(Par.nCellStates + 1:end);

%------------------------------FC-----------------------------------------%

% Layer 1 - Fully connected
ZFc1 = Par.WFc1 * input + Par.bFc1; % ZFc1 = Par.WFc1 * [action; states(1:3)] + Par.bFc1;
ZFc1 = ReLu_function(ZFc1);

% Layer 2 - Fully connected
ZFc2 = Par.WFc2 * ZFc1 + Par.bFc2;
ZFc2 = ReLu_function(ZFc2);

% Layer 3 - Fully connected
ZFc3 = Par.WFc3 * ZFc2 + Par.bFc3;
ZFc3 = ReLu_function(ZFc3);

%---------------------------LSTM------------------------------------------%
z = logistic_function(Par.wz * ZFc3 + Par.Rz * hiddenState + Par.bz);

r = logistic_function(Par.wr * ZFc3 + Par.Rr * hiddenState + Par.br);

h_ = tanh (Par.wh * ZFc3+ r.*Par.Rh* hiddenState+ Par.bh);

hiddenStateNext = z.*hiddenState + (1-z).*h_;

GRU_states = hiddenStateNext;



end

%% Auxiliary Functions

function y = ReLu_function(x)

y = max(0, x);

end


function y = logistic_function(x)

y = 1 ./ (1 + exp(-x));

end

