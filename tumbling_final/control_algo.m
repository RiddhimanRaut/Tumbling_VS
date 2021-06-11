function vc = control_algo(Lsd,error)
lambda=0.0001;
mu=0.00001;
   if(norm(pinv(Lsd))>1000)
        Hsd = Lsd'*Lsd;
        diagHsd = eye(size(Hsd,1)).*Hsd;
        H = inv((mu * diagHsd) + Hsd);
        e =  H * Lsd' *error(:);
        vc = - lambda*e;
   else
        vc = - lambda*pinv(Lsd)*error(:);
   end
end