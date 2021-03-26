function [J, qhat] = compute_energy_functional(dS, dR, qhat, t)

%% Variable declarations
sz = size(dS);
Ji = zeros(1,sz(2));
J = 0;
gamma = 0.1;
eps = 1.0e-5;

%% Energy Functional
for i = 1:sz(2)
    cur_model_err = (norm(dR(t,:)*transpose(qhat(i,:)) - dS(t,i)))^2;
    old_err = (norm(dR(1:t,:)*transpose(qhat(i,:)) - dS(1:t,i)))^2;
    Ji(i) = (cur_model_err + old_err)/2;
end

%% Updated Jacobian vectors
for i = 1:sz(2)
    if(Ji(i) > eps)
        G = [dR*transpose(qhat(i,:)) - dS(:,i); dR(t,:)*transpose(qhat(i,:)) - dS(t,i)];
        H = [dR; dR(t,:)];
        qhat(i,:) = -gamma*transpose(H)*G;
    end
    
end

%% Sum of Ji's
for i = 1:sz(2)
    J = J + Ji(i);
end



end