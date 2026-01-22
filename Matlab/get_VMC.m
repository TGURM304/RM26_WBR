clc
clear

syms theta_m1 theta_m2 alpha gamma theta1 theta2 xe ye  L_a L_b L1 L2

xe = L1*cos(theta1)+L2*cos(theta1+theta2);
ye = L1*sin(theta1)+L2*sin(theta1+theta2);

rules = {
    theta1, theta_m1;
    theta2, (alpha+gamma)/2;
    alpha, theta_m1 - theta_m2;
    gamma, acos((2*L_b^2-2*L_a^2*(1-cos(theta_m1-theta_m2)))/(2*L_b^2));
};

eqations = [xe,ye];
for i = 1:size(rules,1)
    eqations = subs(eqations,rules(i,1),rules(i,2));
end
disp(eqations);
disp(xe);
disp(ye);

vars = [theta_m1 theta_m2];
J = jacobian(eqations,vars);
disp(J(1,1));
disp(J(1,2));
disp(J(2,1));
disp(J(2,2));