syms Dt wx wy wz q0 q1 q2 q3 Q_gyro
syms p00 p01 p02 p03
syms p10 p11 p12 p13
syms p20 p21 p22 p23
syms p30 p31 p32 p33
syms gx gy gz mx my mz
syms Rgyro Rmag
%%%%%%%%%%% gyro variance %%%%%%
G_w = [Q_gyro   0      0;
      0       Q_gyro   0;
      0         0   Q_gyro];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R = [Rgyro  , 0       ,     0 ,     0   ,     0 , 0;
         0  ,    Rgyro,     0 ,     0   ,     0 , 0;
         0  , 0       , Rgyro ,     0   ,     0 , 0;
         0  , 0       ,     0 ,  Rmag   ,     0 , 0;
         0  , 0       ,     0 ,     0   ,  Rmag , 0;
         0  , 0       ,     0 ,     0   ,     0 , Rmag];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = [p00 p01 p02 p03;
     p10 p11 p12 p13;
     p20 p21 p22 p23;
     p30 p31 p32 p33];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
W = [-q1  -q2  -q3;
      q0  -q3   q2;
      q3   q0   q2;
     -q2   q1   q0];
W = Dt/2*W;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F = [ 1,          -Dt/2*wx,           -Dt/2*wy,         -Dt/2*wz;
     Dt/2*wx,         1,               Dt/2*wz,        - Dt/2*wy;
     Dt/2*wy,         -Dt/2*wz,            1,           Dt/2*wx;
     Dt/2*wz,          Dt/2*wy,        -Dt/2*wx,             1 ];
 
 
 H =   2*[gx*q0 + gy*q3 - gz*q2 , gx*q1 + gy*q2 + gz*q3 , -gx*q2 + gy*q1 - gz*q0 , -gx*q3 + gy*q0 + gz*q1;
         -gx*q3 + gy*q0 + gz*q1 , gx*q2 - gy*q1 + gz*q0 ,  gx*q1 + gy*q2 + gz*q3 , -gx*q0 - gy*q3 + gz*q2;
          gx*q2 - gy*q1 + gz*q0 , gx*q3 - gy*q0 - gz*q1 ,  gx*q0 + gy*q3 - gz*q2 ,  gx*q1 + gy*q2 + gz*q3;
          mx*q0 + my*q3 - mz*q2 , mx*q1 + my*q2 + mz*q3 , -mx*q2 + my*q1 - mz*q0 , -mx*q3 + my*q0 + mz*q1;
         -mx*q3 + my*q0 + mz*q1 , mx*q2 - my*q1 + mz*q0 ,  mx*q1 + my*q2 + mz*q3 , -mx*q0 - my*q3 + mz*q2;
          mx*q2 - my*q1 + mz*q0 , mx*q3 - my*q0 - mz*q1 ,  mx*q0 + my*q3 - mz*q2 ,  mx*q1 + my*q2 + mz*q3];

S = H*P*transpose(H) + R;
K = P*transpose(H)/S;
disp(K)
disp(1)
%disp(a(36))
       

