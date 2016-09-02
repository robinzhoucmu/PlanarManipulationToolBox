rng(1);
sampledindices = datasample(1:1:size(all_wrenches_local,1),20,'Replace',false);

[v_s, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local(sampledindices,:)', ...
    all_twists_local_normalized(sampledindices,:)', 5, 50, 1, 1);

[v_s, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local(sampledindices,:)', ...
    all_twists_local_normalized(sampledindices,:)', 5, 50, 5, 0, 1);


[A_s, xi, delta, pred_V, s] = FitElipsoidForceVelocityCVX(all_wrenches_local(sampledindices,:)', ...
        all_twists_local_normalized(sampledindices,:)', 5, 50, 1, 1);
