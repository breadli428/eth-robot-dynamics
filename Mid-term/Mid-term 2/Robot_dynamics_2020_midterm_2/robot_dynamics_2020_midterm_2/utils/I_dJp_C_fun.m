function I_dJp_C = I_dJp_C_fun(in1,in2)
%I_DJP_C_FUN
%    I_DJP_C = I_DJP_C_FUN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    04-Nov-2020 09:21:04

dq1 = in2(1,:);
dq2 = in2(2,:);
dq3 = in2(3,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = q2+q3;
t7 = cos(t6);
t8 = sin(t6);
t9 = t3.*5.404319552844595e+16;
t10 = t5.*5.404319552844595e+16;
t11 = t7.*2.669825385884536e+16;
t12 = t8.*2.669825385884536e+16;
t13 = t7.*4.382926039051794e+16;
t14 = t8.*4.382926039051794e+16;
t16 = t8.*4.866025403784439e-1;
t17 = t7.*2.964101615137755e-1;
t15 = -t12;
t18 = t11+t14;
t22 = dq3.*t2.*(t12-t13).*(-1.110223024625157e-17);
t23 = dq3.*t4.*(t12-t13).*(-1.110223024625157e-17);
t24 = (dq3.*t2.*(t12-t13))./9.007199254740992e+16;
t19 = t13+t15;
t20 = t10+t18;
t21 = t9+t19;
I_dJp_C = reshape([(dq3.*t2.*t18)./9.007199254740992e+16+(dq2.*t2.*t20)./9.007199254740992e+16+(dq1.*t4.*t21)./9.007199254740992e+16,dq1.*t2.*t21.*(-1.110223024625157e-17)+(dq3.*t4.*t18)./9.007199254740992e+16+(dq2.*t4.*t20)./9.007199254740992e+16,0.0,t23+(dq1.*t2.*t20)./9.007199254740992e+16+(dq2.*t4.*t21)./9.007199254740992e+16,t24+(dq1.*t4.*t20)./9.007199254740992e+16-(dq2.*t2.*t21)./9.007199254740992e+16,-dq3.*(t16+t17)-dq2.*(t5.*(3.0./5.0)+t16+t17),t23+(dq1.*t2.*t18)./9.007199254740992e+16-(dq2.*t4.*(t12-t13))./9.007199254740992e+16,t24+(dq1.*t4.*t18)./9.007199254740992e+16+(dq2.*t2.*(t12-t13))./9.007199254740992e+16,(dq2+dq3).*(t7.*5.339650771769071e+15+t8.*8.765852078103587e+15).*(-5.551115123125783e-17)],[3,3]);
