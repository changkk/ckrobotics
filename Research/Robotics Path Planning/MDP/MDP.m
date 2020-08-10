clear all;
V=[0, 0, 0];
gamma = 0.5;
flag = 0;

while flag == 0

    v11=0.7*(3+gamma*V(1))+0.3*(0+gamma*V(2)); % Top - On
    v12=0.5*(4+gamma*V(1))+0.5*(1+gamma*V(2)); % Top - Off

    v21=0.4*(3+gamma*V(1))+0.4*(0+gamma*V(2))+0.2*(-1+gamma*V(3));  % Middle - On
    v22=0.5*(1+gamma*V(2))+0.5*(0+gamma*V(3)); % Middle - Off

    v31=0.7*(0+gamma*V(2))+0.3*(-1+gamma*V(3));  % Bottom - On
    v32=1*(0+gamma*V(3)); % Bottom - Off

    V_prev = V;
    [V(1),policy(1)]=max([v11,v12]);
    [V(2),policy(2)]=max([v21,v22]);
    [V(3),policy(3)]=max([v31,v32]);

    if(abs((V_prev(1)-V(1)))<0.001 && abs((V_prev(2)-V(2)))<0.001 && abs((V_prev(3)-V(3)))<0.001)
        flag = 1;
    end

end
V
policy