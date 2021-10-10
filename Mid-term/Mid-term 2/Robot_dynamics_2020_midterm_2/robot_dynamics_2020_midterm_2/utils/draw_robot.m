function gc_new = draw_robot(gc, kin, params)

% Compute kinematicss
T_IK_1 = eval(subs(kin.T_Ik{1}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_IK_3 = eval(subs(kin.T_Ik{3}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
I_r_IE = eval(subs(kin.I_r_IE, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
I_r_IC = eval(subs(kin.I_r_IC, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));

p_3Croot_3 = [params.l31; 0; 0];
T_3Croot = [eye(3) p_3Croot_3;
        0 0 0 1];
T_ICroot = T_IK_3 * T_3Croot;   

p1 = T_IK_1(1:3,4);
p3 = T_IK_3(1:3,4);
pE = I_r_IE;
pCroot = T_ICroot(1:3,4);
pC = I_r_IC;

plot3([0.0 p1(1)],[0.0 p1(2)], [0.0 p1(3)],'linewidth',5);
plot3([p1(1) p3(1)],[ p1(2) p3(2)], [ p1(3) p3(3)],'linewidth',5);
plot3([p3(1) pE(1)],[ p3(2) pE(2)], [ p3(3) pE(3)],'linewidth',5);
plot3([pCroot(1) pC(1)],[pCroot(2) pC(2)], [pCroot(3) pC(3)],'linewidth',5);

width = 3.0;
height = 2.0;
xlim([-width/2, width/2])
ylim([-width/2, width/2])
zlim([0.0,height])
hold off

gc_new = gc;
end