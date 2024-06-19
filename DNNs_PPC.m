clc
close all
clear all
Time=0.01;stoptime=20;k=1;deltaT=0.01; dt=Time; %j=1; 
gamma_1=10;gamma_2=20;sigma_1=3;omega_1=2;sigma_2=3;omega_2=2;
rho=zeros(1);rho_3=zeros(1);
vartheta=zeros(1);varpi=zeros(1);tau=zeros(0);%design constants(performance)
x_1=zeros(1); x_2=zeros(1); y_d=zeros(1); %states and references with ther initial values
z_1=zeros(1);xi_1=zeros(1);xi_2=zeros(1); %tracking error
C_1=zeros(1);C_2=zeros(1);x_h1=zeros(3,1);

w=zeros(1,1);  dw=zeros(1,1);  %stochastic noise %stochastic for white noise

cost_1=zeros(1); cost_2=zeros(1);
V_01=zeros(3,5); V_11=zeros(5,5);V_21=zeros(5,5); V_31=zeros(5,5);V_41=zeros(5,1);%DNN weight update for first subsystems (h)
W_01=zeros(15,1); W_11=zeros(25,1);W_21=zeros(25,1); W_31=zeros(25,1);W_41=zeros(5,1);%DNN weight update for first subsystems (h)的列向量表示形式
Lambda_01=zeros(15,1);Lambda_11=zeros(25,1);Lambda_21=zeros(25,1);Lambda_31=zeros(25,1);Lambda_41=zeros(5,1);
U_01=zeros(3,5); U_11=zeros(5,5);U_21=zeros(5,5); U_31=zeros(5,5);U_41=zeros(5,1);%NN weight for first subsystems(J)
W_51=zeros(15,1); W_61=zeros(25,1);W_71=zeros(25,1); W_81=zeros(25,1);W_91=zeros(5,1);%DNN weight update for first subsystems (J)的列向量表示形式
Lambda_51=zeros(15,1);Lambda_61=zeros(25,1);Lambda_71=zeros(25,1); Lambda_81=zeros(25,1);Lambda_91=zeros(5,1);
v_01(k)=zeros(1);v_11(k)=zeros(1); v_21(k)=zeros(1); v_31(k)=zeros(1); v_41(k)=zeros(1);% NN weight norm for first subsystems(h)
u_01(k)=zeros(1);u_11(k)=zeros(1); u_21(k)=zeros(1); u_31(k)=zeros(1); u_41(k)=zeros(1);% NN weight norm for first subsystems(J)

V_02=zeros(4,5); V_12=zeros(5,5);V_22=zeros(5,5); V_32=zeros(5,5);V_42=zeros(5,1);%DNN weight update for  the second  subsystems (h)
W_02=zeros(20,1); W_12=zeros(25,1);W_22=zeros(25,1); W_32=zeros(25,1);W_42=zeros(5,1);%DNN weight update for the second subsystems (h)的列向量表示形式
Lambda_02=zeros(20,1);Lambda_12=zeros(25,1);Lambda_22=zeros(25,1);Lambda_32=zeros(25,1);Lambda_42=zeros(5,1);
U_02=zeros(4,5); U_12=zeros(5,5);U_22=zeros(5,5); U_32=zeros(5,5);U_42=zeros(5,1);%NN weight for  the second  subsystems(J)
W_52=zeros(20,1); W_62=zeros(25,1);W_72=zeros(25,1); W_82=zeros(25,1);W_92=zeros(5,1);%DNN weight update for the second subsystems (J)的列向量表示形式
Lambda_52=zeros(20,1);Lambda_62=zeros(25,1);Lambda_72=zeros(25,1); Lambda_82=zeros(25,1);Lambda_92=zeros(5,1);
v_02(k)=zeros(1);v_12(k)=zeros(1); v_22(k)=zeros(1); v_32(k)=zeros(1); v_42(k)=zeros(1);% NN weight norm for  the second  subsystems(h)
u_02(k)=zeros(1);u_12(k)=zeros(1); u_22(k)=zeros(1); u_32(k)=zeros(1); u_42(k)=zeros(1);% NN weight norm for  the second  subsystems(J)

%第一个子系统权重更新定律V的参数
I01=eye(5);
I11=eye(5);
I21=eye(5);
I31=eye(5);
I41=eye(1);
%学习率
T01=0.4*eye(15);
T11=0.4*eye(25);
T21=0.4*eye(25);
T31=0.4*eye(25);
T41=0.4*eye(5);
%第一个子系统权重更新定律U的参数
I51=eye(5);
I61=eye(5);
I71=eye(5);
I81=eye(5);
I91=eye(1);
%学习率
T51=0.4*eye(15);
T61=0.4*eye(25);
T71=0.4*eye(25);
T81=0.4*eye(25);
T91=0.4*eye(5);

%第二个子系统权重更新定律V的参数
I02=eye(5);
I12=eye(5);
I22=eye(5);
I32=eye(5);
I42=eye(1);
%学习率
T02=0.5*eye(20);
T12=0.5*eye(25);
T22=0.5*eye(25);
T32=0.5*eye(25);
T42=0.5*eye(5);

%第二个子系统权重更新定律U的参数
I52=eye(5);
I62=eye(5);
I72=eye(5);
I82=eye(5);
I92=eye(1);
%学习率
T52=0.5*eye(20);
T62=0.5*eye(25);
T72=0.5*eye(25);
T82=0.5*eye(25);
T92=0.5*eye(5);

alpha=zeros(1);u=zeros(1); %the virtual and actual controller
totalCost1=zeros(1);

 

for t=0:Time:stoptime
    randn('state',100)
    if k==1
        dw(k)=sqrt(dt)*randn;       % initial values for stochastic process
        w(k)=dw(k);                                    % W(0) = 0 不允许，所以首先置值
    end
    if k<=stoptime/Time
        dw(k)=sqrt(dt)*randn;   % 产生序列
        w(k+1)=w(k)+dw(k);
    end
        x_1(1)=2; x_2(1)=3;y_d(1)=0;rho(1)=4;rho_3(1)=-1.9;
        for i=1:15
            W_01(i,1)=0.2;
            W_51(i,1)=0.5;
       end
        for i=1:25
            W_11(i,1)=0.2;
            W_21(i,1)=0.2;
            W_31(i,1)=0.2;
            W_61(i,1)=0.5;
            W_71(i,1)=0.5;
            W_81(i,1)=0.5;
        end
        for i=1:5
            W_41(i,1)=0.2;
            W_91(i,1)=0.5;
        end
        for i=1:20
            W_02(i,1)=0.5;
            W_52(i,1)=0.3;
        end
        for i=1:25
            W_12(i,1)=0.5;
            W_22(i,1)=0.5;
            W_32(i,1)=0.5;
            W_62(i,1)=0.3;
            W_72(i,1)=0.3;
            W_82(i,1)=0.3;
        end
        for i=1:5
            W_42(i,1)=0.5;
            W_92(i,1)=0.3;
        end
   
    
    V_01= reshape(W_01(:,k), 3, 5);
    V_11= reshape(W_11(:,k), 5, 5);
    V_21= reshape(W_21(:,k), 5, 5);
    V_31= reshape(W_31(:,k), 5, 5);
    V_41= reshape(W_41(:,k), 5, 1);
    U_01= reshape(W_51(:,k), 3, 5);
    U_11= reshape(W_61(:,k), 5, 5);
    U_21= reshape(W_71(:,k), 5, 5);
    U_31= reshape(W_81(:,k), 5, 5);
    U_41= reshape(W_91(:,k), 5, 1);
    V_02= reshape(W_02(:,k), 4, 5);
    V_12= reshape(W_12(:,k), 5, 5);
    V_22= reshape(W_22(:,k), 5, 5);
    V_32= reshape(W_32(:,k), 5, 5);
    V_42= reshape(W_42(:,k), 5, 1);
    U_02= reshape(W_52(:,k), 4, 5);
    U_12= reshape(W_62(:,k), 5, 5);
    U_22= reshape(W_72(:,k), 5, 5);
    U_32= reshape(W_82(:,k), 5, 5);
    U_42= reshape(W_92(:,k), 5, 1);
   
    
    z_1(k)=x_1(k)-y_d(k);
   
    vartheta(k)=z_1(k)/rho(k);

    xi_1(k)=1/2*log((vartheta(k)+0.9)/(0.9-vartheta(k)));
    x_h1=[x_1(k);xi_1(k);1];
    B91=U_41'*diag(1-tanh(U_31'*tanh(U_21'*tanh(U_11'*tanh(U_01'*x_h1)))).^2);
    B81=U_31'*diag(1-(tanh(U_21'*tanh(U_11'*tanh(U_01'*x_h1)))).^2);
    B71=U_21'*diag(1-(tanh(U_11'*tanh(U_01'*x_h1))).^2);
    B61=U_11'*diag(1-(tanh(U_01'*x_h1)).^2);
    B41=V_41'*diag(1-tanh(V_31'*tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1)))).^2);
    B31=V_31'*diag(1-(tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1)))).^2);
    B21=V_21'*diag(1-(tanh(V_11'*tanh(V_01'*x_h1))).^2);
    B11=V_11'*diag(1-(tanh(V_01'*x_h1)).^2);
    C91=tanh(U_31'*tanh(U_21'*tanh(U_11'*tanh(U_01'*x_h1))));
    C81=tanh(U_21'*tanh(U_11'*tanh(U_01'*x_h1)));
    C71=tanh(U_11'*tanh(U_01'*x_h1));
    C61=tanh(U_01'*x_h1);
    C41=tanh(V_31'*tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1))));
    C31=tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1)));
    C21=tanh(V_11'*tanh(V_01'*x_h1));
    C11=tanh(V_01'*x_h1);
    D01=kron(I01,x_h1');
    D11=kron(I11,C11');
    D21=kron(I21,C21');
    D31=kron(I31,C31');
    D41=kron(I41,C41');
    D51=kron(I51,x_h1');
    D61=kron(I61,C61');
    D71=kron(I71,C71');
    D81=kron(I81,C81');
    D91=kron(I91,C91');
    %第一个系统的参量
    Lambda_01=(B41*B31*B21*B11*D01)';
    Lambda_11=(B41*B31*B21*D11)';
    Lambda_21=(B41*B31*D21)';
    Lambda_31=(B41*D31)';
    Lambda_41=(D41)';
    Lambda_51=(B91*B81*B71*B61*D51)';
    Lambda_61=(B91*B81*B71*D61)';
    Lambda_71=(B91*B81*D71)';
    Lambda_81=(B91*D81)';
    Lambda_91=(D91)';
   %将第一个系统的所有更新权重矩阵进行按列排列
    W_01(:,k)=V_01(:);W_11(:,k)=V_11(:); W_21(:,k)=V_21(:); W_31(:,k)=V_31(:);W_41(:,k)=V_41(:);
    W_51(:,k)=U_01(:); W_61(:,k)=U_11(:); W_71(:,k)=U_21(:); W_81(:,k)=U_31(:);W_91(:,k)=U_41(:);

    
    C_1=V_41'*tanh(V_31'*tanh(V_21'*tanh(V_11'*tanh(V_01'*x_h1))));%H_1函数的估计
    C_2=U_41'*tanh(U_31'*tanh(U_21'*tanh(U_11'*tanh(U_01'*x_h1))));%J_1函数的估计
    tau(k)=1/(2*rho(k))*(1/(vartheta(k)+0.9)-1/(vartheta(k)-0.9));
    alpha(k)=-gamma_1*xi_1(k)-3/4*(tau(k)^(1/3))*xi_1(k)-C_1-1/2*C_2+z_1(k)*rho_3(k)/rho(k);%virtual controller
    xi_2(k)=x_2(k)-alpha(k); 

    x_h2=[x_1(k);x_2(k);xi_2(k);1];
    B92=U_42'*diag(1-tanh(U_32'*tanh(U_22'*tanh(U_12'*tanh(U_02'*x_h2)))).^2);
    B82=U_32'*diag(1-(tanh(U_22'*tanh(U_12'*tanh(U_02'*x_h2)))).^2);
    B72=U_22'*diag(1-(tanh(U_12'*tanh(U_02'*x_h2))).^2);
    B62=U_12'*diag(1-(tanh(U_02'*x_h2)).^2);
    B42=V_42'*diag(1-(tanh(V_32'*tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2)))).^2));
    B32=V_32'*diag(1-(tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2)))).^2);
    B22=V_22'*diag(1-(tanh(V_12'*tanh(V_02'*x_h2))).^2);
    B12=V_12'*diag(1-(tanh(V_02'*x_h2)).^2);
    C92=tanh(U_32'*tanh(U_22'*tanh(U_12'*tanh(U_02'*x_h2))));
    C82=tanh(U_22'*tanh(U_12'*tanh(U_02'*x_h2)));
    C72=tanh(U_12'*tanh(U_02'*x_h2));
    C62=tanh(U_02'*x_h2);
    C42=tanh(V_32'*tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2))));
    C32=tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2)));
    C22=tanh(V_12'*tanh(V_02'*x_h2));
    C12=tanh(V_02'*x_h2);
    D02=kron(I02,x_h2');
    D12=kron(I12,C12');
    D22=kron(I22,C22');
    D32=kron(I32,C32');
    D42=kron(I42,C42');
    D52=kron(I52,x_h2');
    D62=kron(I62,C62');
    D72=kron(I72,C72');
    D82=kron(I82,C82');
    D92=kron(I92,C92');
   
    %第一个系统的参量
    Lambda_02=(B42*B32*B22*B12*D02)';
    Lambda_12=(B42*B32*B22*D12)';
    Lambda_22=(B42*B32*D22)';
    Lambda_32=(B42*D32)';
    Lambda_42=(D42)';
    Lambda_52=(B92*B82*B72*B62*D52)';
    Lambda_62=(B92*B82*B72*D62)';
    Lambda_72=(B92*B82*D72)';
    Lambda_82=(B92*D82)';
    Lambda_92=(D92)';
  %将第一个系统的所有更新权重矩阵进行按列排列
    W_02(:,k)=V_02(:);W_12(:,k)=V_12(:); W_22(:,k)=V_22(:); W_32(:,k)=V_32(:);W_42(:,k)=V_42(:);
    W_52(:,k)=U_02(:); W_62(:,k)=U_12(:); W_72(:,k)=U_22(:); W_82(:,k)=U_32(:);W_92(:,k)=U_42(:);
    
   
    C_3=V_42'*tanh(V_32'*tanh(V_22'*tanh(V_12'*tanh(V_02'*x_h2))));%H_2函数的估计
    C_4=U_42'*tanh(U_32'*tanh(U_22'*tanh(U_12'*tanh(U_02'*x_h2))));%J_2函数的估计
    u(k)=-gamma_2*xi_2(k)-C_3-1/2*C_4;%actual controller
    
v_01(k)=norm(W_01(:,k));v_11(k)=norm(W_11(:,k)); v_21(k)=norm(W_21(:,k)); v_31(k)=norm(W_31(:,k)); v_41(k)=norm(W_41(:,k));%weight matrice norm
u_01(k)=norm(W_51(:,k));u_11(k)=norm(W_61(:,k)); u_21(k)=norm(W_71(:,k)); u_31(k)=norm(W_81(:,k)); u_41(k)=norm(W_91(:,k));%weight matrice norm
cost_1(k)=xi_1(k)^2+alpha(k)^2;%cost function 
    
  if k<=stoptime/Time 
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  W_01(:,k+1)=W_01(:,k)+deltaT*(T01*(tau(k).*Lambda_01*xi_1(k)^3-sigma_1*W_01(:,k)));
  W_11(:,k+1)=W_11(:,k)+deltaT*(T11*(tau(k).*Lambda_11*xi_1(k)^3-sigma_1*W_11(:,k))); 
  W_21(:,k+1)=W_21(:,k)+deltaT*(T21*(tau(k).*Lambda_21*xi_1(k)^3-sigma_1*W_21(:,k)));
  W_31(:,k+1)=W_31(:,k)+deltaT*(T31*(tau(k).*Lambda_31*xi_1(k)^3-sigma_1*W_31(:,k)));
  W_41(:,k+1)=W_41(:,k)+deltaT*(T41*(tau(k).*Lambda_41*xi_1(k)^3-sigma_1*W_41(:,k))); 
  W_51(:,k+1)=W_51(:,k)+deltaT*1/2*(T51*(tau(k).*Lambda_51*xi_1(k)^3-omega_1*W_51(:,k)));
  W_61(:,k+1)=W_61(:,k)+deltaT*1/2*(T61*(tau(k).*Lambda_61*xi_1(k)^3-omega_1*W_61(:,k)));
  W_71(:,k+1)=W_71(:,k)+deltaT*1/2*(T71*(tau(k).*Lambda_71*xi_1(k)^3-omega_1*W_71(:,k))); 
  W_81(:,k+1)=W_81(:,k)+deltaT*1/2*(T81*(tau(k).*Lambda_81*xi_1(k)^3-omega_1*W_81(:,k)));
  W_91(:,k+1)=W_91(:,k)+deltaT*1/2*(T91*(tau(k).*Lambda_91*xi_1(k)^3-omega_1*W_91(:,k)));
  end

%%%%%%%%%
v_02(k)=norm(W_02(:,k));v_12(k)=norm(W_12(:,k)); v_22(k)=norm(W_22(:,k)); v_32(k)=norm(W_32(:,k)); v_42(k)=norm(W_42(:,k));%weight matrice norm
u_02(k)=norm(W_52(:,k));u_12(k)=norm(W_62(:,k)); u_22(k)=norm(W_72(:,k)); u_32(k)=norm(W_82(:,k)); u_42(k)=norm(W_92(:,k));%weight matrice norm
cost_2(k)=xi_2(k)^2+u(k)^2;%cost function
totalCost1(k)=cost_1(k)+cost_2(k);%total cost function
if k<=stoptime/Time 
  W_02(:,k+1)=W_02(:,k)+deltaT.*(T02*(Lambda_02*xi_2(k)^3-sigma_2*W_02(:,k)));
  W_12(:,k+1)=W_12(:,k)+deltaT.*(T12*(Lambda_12*xi_2(k)^3-sigma_2*W_12(:,k))); 
  W_22(:,k+1)=W_22(:,k)+deltaT.*(T22*(Lambda_22*xi_2(k)^3-sigma_2*W_22(:,k)));
  W_32(:,k+1)=W_32(:,k)+deltaT.*(T32*(Lambda_32*xi_2(k)^3-sigma_2*W_32(:,k)));
  W_42(:,k+1)=W_42(:,k)+deltaT.*(T42*(Lambda_42*xi_2(k)^3-sigma_2*W_42(:,k))); 
  W_52(:,k+1)=W_52(:,k)+deltaT*1/2.*(T52*(Lambda_52*xi_2(k)^3-omega_2*W_52(:,k)));
  W_62(:,k+1)=W_62(:,k)+deltaT*1/2.*(T62*(Lambda_62*xi_2(k)^3-omega_2*W_62(:,k)));
  W_72(:,k+1)=W_72(:,k)+deltaT*1/2.*(T72*(Lambda_72*xi_2(k)^3-omega_2*W_72(:,k))); 
  W_82(:,k+1)=W_82(:,k)+deltaT*1/2.*(T82*(Lambda_82*xi_2(k)^3-omega_2*W_82(:,k)));
  W_92(:,k+1)=W_92(:,k)+deltaT*1/2.*(T92*(Lambda_92*xi_2(k)^3-omega_2*W_92(:,k)));
  
  y_d(k+1)=5*sin(0.5*t); %refeference
  rho(k+1)=3.8*exp(-0.5*t)+0.2;
  rho_3(k+1)=-1.9*exp(-0.5*t);

  x_1(k+1)=x_1(k)+deltaT*(x_1(k)*sin(x_1(k))+x_2(k))+cos(x_1(k))^2*dw(k);
  x_2(k+1)=x_2(k)+deltaT*(x_2(k)*cos(x_1(k))+u(k))+sin(x_1(k))*cos(x_2(k))*dw(k);
 
  
  
 
end
k=k+1;
end

t=0:Time:stoptime;


figure
plot(t,y_d,'--r'); 
hold on
plot(t,x_1,'b');
xlabel('Time');
legend('y_d', 'x_1');
axis([0 20 -6 6]);
hold off



figure
plot(t,z_1,'--r'); 
hold on
plot(t,0.9*rho,'b');
plot(t,-0.9*rho,'b');
xlabel('Time');
legend('tracking error', 'upper bound','lower bound');
axis([0 20 -4 4]);
hold off



figure
subplot(2,1,1)
plot(t,cost_1(:),'b','LineWidth', 0.05); xlabel('Cost function c_1(z_{1},\alpha_{1}) of the 1st step');
axis([0 20 0 200]);
subplot(2,1,2)
plot(t,cost_2(:),'b','LineWidth', 0.05); xlabel('Cost function c_2(z_{2},u) of the 2nd step');
axis([0 20 0 2000]);

figure
plot(t,v_01(:),'b'); 
hold on
plot(t,v_11(:),'r'); 
plot(t,v_21(:),'g'); 
plot(t,v_31(:),'c');
plot(t,v_41(:),'m');
legend('v_{01}', 'v_{11}', 'v_{21}', 'v_{31}', 'v_{41}');
axis([0 20 0 2.5]);
hold off

figure
plot(t,u_01(:),'b'); 
hold on
plot(t,u_11(:),'r'); 
plot(t,u_21(:),'g'); 
plot(t,u_31(:),'c');
plot(t,u_41(:),'m');
xlabel('J DNN weight for the 1st step');
legend('u_{01}', 'u_{11}', 'u_{21}', 'u_{31}', 'u_{41}');
axis([0 20 0 2.5]);
hold off

figure
plot(t,v_02(:),'b'); 
hold on
plot(t,v_12(:),'r'); 
plot(t,v_22(:),'g'); 
plot(t,v_32(:),'c');
plot(t,v_42(:),'m');% 特殊点的 x 坐标
xlabel('h DNN weight for the 2st step');
legend('v_{02}', 'v_{12}', 'v_{22}', 'v_{32}', 'v_{42}');
axis([0 20 0 40]);
hold off


figure
plot(t,u_02(:),'b'); 
hold on
plot(t,u_12(:),'r'); 
plot(t,u_22(:),'g'); 
plot(t,u_32(:),'c');
plot(t,u_42(:),'m');
xlabel('J DNN weight for the 2st step');
legend('u_{02}', 'u_{12}', 'u_{22}', 'u_{32}', 'u_{42}');
axis([0 20 0 90]);
hold off


