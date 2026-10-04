//  % ********************** Filter Propagation **********************
//  CS = [cos(X_INS(5)) sin(X_INS(5))];
//  Ad_E = [ 
//          1           0           0   0   0   dtKalman*CS(1)          -dtKalman*CS(2)             0;
//          0           1           0   0   0   dtKalman*CS(2)          dtKalman*CS(1)              0;
//       dtKalman       0           1   0   0   (dtKalman^2)*CS(1)*0.5  (dtKalman^2)*CS(2)*(-0.5)   0;
//          0        dtKalman       0   1   0   (dtKalman^2)*CS(2)*0.5  (dtKalman^2)*CS(1)*0.5      0;
//          0           0           0   0   1   0                        0                          dtKalman;
//          0           0           0   0   0   1                        0                          0;
//          0           0           0   0   0   0                        1                          0;
//          0           0           0   0   0   0                        0                          1
//      ];
//
//  Bd_E = [ 
//          dtKalman*CS(1)               -dtKalman*CS(2)                 0;
//          dtKalman*CS(2)               dtKalman*CS(1)                  0;
//          (dtKalman^2)*CS(1)*0.5       (dtKalman^2)*CS(2)*(-0.5)       0;
//          (dtKalman^2)*CS(2)*0.5       (dtKalman^2)*CS(1)*0.5          0;
//          0                            0                               dtKalman;
//          0                            0                               0;
//          0                            0                               0;
//          0                            0                               0
//          ];
//
//  P = Ad_E * P * Ad_E' + Bd_E * Q * Bd_E' + beta*P0;
//  % ***********************************************************

//  % ********************** Filter Update **********************
//  Y_E = X_INS(3:5)  - [Px_LS; Py_LS; Psi_COMP];
//
//  G = P * (H') * inv(H * P * (H') + R);
//
//  P = (eye(8) - G*H) * P;
//
//  Xhat_E = [ zeros(5,1); BiasHat] + G * Y_E;
//
//  X_INS = X_INS - Xhat_E(1:5);
//
//  BiasHat = Xhat_E(6:8);
//  % ***********************************************************

void EKF_initialization()
{
  Bias[0]=0; Bias[1]=0; Bias[2]=0;
  
  X_INS[0]=0;X_INS[1]=0;X_INS[2]=0;X_INS[3]=0;X_INS[4]=0;

  R[0][0]= noise_px_LS*noise_px_LS  ;R[0][1]= 0                         ;R[0][2]= 0; 
  R[1][0]= 0                        ;R[1][1]= noise_py_LS*noise_py_LS   ;R[1][2]= 0;
  R[2][0]= 0                        ;R[2][1]= 0                         ;R[2][2]= noise_psi_comp*noise_psi_comp;
  
  Q[0][0]= noise_ax_INS*noise_ax_INS  ;Q[0][1]= 0                           ;Q[0][2]= 0; 
  Q[1][0]= 0                          ;Q[1][1]= noise_ay_INS*noise_ay_INS   ;Q[1][2]= 0;
  Q[2][0]= 0                          ;Q[2][1]= 0                           ;Q[2][2]= noise_wz_INS*noise_wz_INS;
  
  H[0][0]= 0 ;H[0][1]=  0 ;H[0][2]= 1 ;H[0][3]= 0 ;H[0][4]= 0 ;H[0][5]= 0 ;H[0][6]= 0 ;H[0][7]= 0 ;
  H[1][0]= 0 ;H[1][1]=  0 ;H[1][2]= 0 ;H[1][3]= 1 ;H[1][4]= 0 ;H[1][5]= 0 ;H[1][6]= 0 ;H[1][7]= 0 ;
  H[2][0]= 0 ;H[2][1]=  0 ;H[2][2]= 0 ;H[2][3]= 0 ;H[2][4]= 1 ;H[2][5]= 0 ;H[2][6]= 0 ;H[2][7]= 0 ;
  
  byte i,j;
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  Ident[i][j]=0; } }
  for(i=0;i<8;i++){ Ident[i][i]=1; }

  Matrix.Copy((float*)Ident, 8, 8, (float*)P0);
  P0[0][0]=50; P0[1][1]=50; P0[2][2]=50; P0[3][3]=50; P0[4][4]=deg2rad(10); P0[5][5]=10; P0[6][6]=10; P0[8][8]=10;
  
  Matrix.Copy((float*)P0, 8, 8, (float*)P);

  //from this moment on, the P0 matrix will be beta*P0 (the matrix is replaced)
  Matrix.Scale((float*) P0, 8, 8, beta);
}

void EKF_propagation()
{ 
  //Because the propagation takes 26ms, I'm going to split it in 2 steps
  if (prop_step==1)
  {
//    digitalWrite(debugPin1, HIGH); //Debug on osciliscope
    clear_temporary_matrices(); //440us
    
    rotate_body2inertial(); //370us
    
    //U_INS = U - BIAS
    U[2]=deg2rad(wz_b);
    U[0]=U[0]-Bias[0];
    U[1]=U[1]-Bias[1];
    U[2]=U[2]-Bias[2];
    
    //*** X_INS(k+1) = f(X_INS(k),U_INS(k)) ***
    euler_integration2(); //100us
  
    // from now on it takes ~ 23ms
    float cos_psi = cos(X_INS[4]); //psi in [radians]
    float sin_psi = sin(X_INS[4]);
  
    Ad_E[0][0]=    1    ;Ad_E[0][1]=    0    ;Ad_E[0][2]=   0   ;Ad_E[0][3]=   0   ;Ad_E[0][4]=    0   ;Ad_E[0][5]=     dtK*cos_psi        ;Ad_E[0][6]=    -dtK*sin_psi          ;Ad_E[0][7]=    0;
    Ad_E[1][0]=    0    ;Ad_E[1][1]=    1    ;Ad_E[1][2]=   0   ;Ad_E[1][3]=   0   ;Ad_E[1][4]=    0   ;Ad_E[1][5]=     dtK*sin_psi        ;Ad_E[1][6]=     dtK*cos_psi          ;Ad_E[1][7]=    0;
    Ad_E[2][0]=   dtK   ;Ad_E[2][1]=    0    ;Ad_E[2][2]=   1   ;Ad_E[2][3]=   0   ;Ad_E[2][4]=    0   ;Ad_E[2][5]=  dtK*dtK*cos_psi*0.5   ;Ad_E[2][6]= dtK*dtK*sin_psi*(-0.5)   ;Ad_E[2][7]=    0;
    Ad_E[3][0]=    0    ;Ad_E[3][1]=   dtK   ;Ad_E[3][2]=   0   ;Ad_E[3][3]=   1   ;Ad_E[3][4]=    0   ;Ad_E[3][5]=  dtK*dtK*sin_psi*0.5   ;Ad_E[3][6]=  dtK*dtK*cos_psi*0.5     ;Ad_E[3][7]=    0;
    Ad_E[4][0]=    0    ;Ad_E[4][1]=    0    ;Ad_E[4][2]=   0   ;Ad_E[4][3]=   0   ;Ad_E[4][4]=    1   ;Ad_E[4][5]=           0            ;Ad_E[4][6]=            0             ;Ad_E[4][7]=   dtK;
    Ad_E[5][0]=    0    ;Ad_E[5][1]=    0    ;Ad_E[5][2]=   0   ;Ad_E[5][3]=   0   ;Ad_E[5][4]=    0   ;Ad_E[5][5]=           1            ;Ad_E[5][6]=            0             ;Ad_E[5][7]=    0;
    Ad_E[6][0]=    0    ;Ad_E[6][1]=    0    ;Ad_E[6][2]=   0   ;Ad_E[6][3]=   0   ;Ad_E[6][4]=    0   ;Ad_E[6][5]=           0            ;Ad_E[6][6]=            1             ;Ad_E[6][7]=    0;
    Ad_E[7][0]=    0    ;Ad_E[7][1]=    0    ;Ad_E[7][2]=   0   ;Ad_E[7][3]=   0   ;Ad_E[7][4]=    0   ;Ad_E[7][5]=           0            ;Ad_E[7][6]=            0             ;Ad_E[7][7]=    1;
  
    Bd_E[0][0]=    dtK*cos_psi         ;Bd_E[0][1]=     -dtK*sin_psi         ;Bd_E[0][2]=   0;
    Bd_E[1][0]=    dtK*sin_psi         ;Bd_E[1][1]=      dtK*cos_psi         ;Bd_E[1][2]=   0;
    Bd_E[2][0]=  dtK*dtK*cos_psi*0.5   ;Bd_E[2][1]=  dtK*dtK*sin_psi*(-0.5)  ;Bd_E[2][2]=   0;
    Bd_E[3][0]=  dtK*dtK*sin_psi*0.5   ;Bd_E[3][1]=   dtK*dtK*cos_psi*0.5    ;Bd_E[3][2]=   0;
    Bd_E[4][0]=        0               ;Bd_E[4][1]=          0               ;Bd_E[4][2]=  dtK;
    Bd_E[5][0]=        0               ;Bd_E[5][1]=          0               ;Bd_E[5][2]=   0;
    Bd_E[6][0]=        0               ;Bd_E[6][1]=          0               ;Bd_E[6][2]=   0;
    Bd_E[7][0]=        0               ;Bd_E[7][1]=          0               ;Bd_E[7][2]=   0;
    
    //***  P = Ad_E * P * Ad_E' + Bd_E * Q * Bd_E' + beta*P0; ***
    //  t1 <- A*P       8x8 output order
    //  t2 <- A'        8x8
    //  t3 <- t1 * t2   8x8
    //  t4 <- B*Q       8x3 output order
    //  t5 <- B'        3x8
    //  t6 <- t4*t5     8x8
    //  t1 <- t3 + t6   8x8
    //  P  <- t1+betaP0 8x8
    Matrix.Multiply((float*)Ad_E, (float*)P, 8, 8, 8, (float*)t1);
    Matrix.Transpose((float*)Ad_E, 8, 8, (float*)t2);
  }
  if(prop_step==2)
  {
    Matrix.Multiply((float*)t1, (float*)t2, 8, 8, 8, (float*)t3);
    Matrix.Multiply((float*)Bd_E, (float*)Q, 8, 3, 3, (float*)t4);
    Matrix.Transpose((float*)Bd_E, 8, 3, (float*)t5);
    Matrix.Multiply((float*)t4, (float*)t5, 8, 3, 8, (float*)t6);
    Matrix.Add((float*)t3, (float*)t6, 8, 8, (float*)t1);
    Matrix.Add((float*)t1, (float*)P0, 8, 8, (float*)P);
  //*** *** *** *** *** *** *** *** *** *** *** *** *** *** ***
      
  //  Matrix.Print((float*)Ad_E, 8, 8, "Ad_E=");
  //  Matrix.Print((float*)Bd_E, 8, 3, "Bd_E=");
  //  Matrix.Print((float*)P, 8, 8, "P=");
//  digitalWrite(debugPin1, LOW); //Debug on osciliscope
  }
}

void EKF_update()
{
  //I'm also dividing the update in two steps
  if(updt_step==1)
  {
//    digitalWrite(debugPin2, HIGH); //Debug on osciliscope
    clear_temporary_matrices(); //440us  
    
    // from now on it takes ~ 23ms
    //*** Y_E = X_INS(3:5)  - [Px_LS; Py_LS; Psi_COMP]; *** 
    Y_E[0] = X_INS[2] - pos_LS[0]; 
    Y_E[1] = X_INS[3] - pos_LS[1]; 
    Y_E[2] = X_INS[4] - deg2rad(actual_HEADING);
  
    //***  G = P * (H') * inv(H * P * (H') + R); ***
    // t1 <- H'       8x3
    // t2 <- P*t1     8x3
    // t3 <- H*P      3x8
    // t4 <- t3*t1    3x3
    // t5 <- t4+R     3x3
    // t5 <- inv(t5)  3x3
    // G  <- t2 * t5  8x3
    Matrix.Transpose((float*)H, 3, 8, (float*)t1);
    Matrix.Multiply((float*)P, (float*)t1, 8, 8, 3, (float*)t2);
    Matrix.Multiply((float*)H, (float*)P, 3, 8, 8, (float*)t3);
    Matrix.Multiply((float*)t3, (float*)t1, 3, 8, 3, (float*)t4);
    Matrix.Add((float*)t4, (float*)R, 3, 3, (float*)t5);
    Matrix.Invert((float*)t5, 3); 
    Matrix.Multiply((float*)t2, (float*)t5, 8, 3, 3, (float*)G);
  }
  if(updt_step==2)
  {
    //***  P = (eye(8) - G*H) * P; ***
    // t1 <- G*H          8x8
    // t2 <- Ident - t1   8x8
    // t3 <- t2*P         8x8
    // P  <- t3           8x8
    Matrix.Multiply((float*)G, (float*)H, 8, 3, 8, (float*)t1);
    Matrix.Subtract((float*)Ident, (float*)t1, 8, 8, (float*)t2);
    Matrix.Multiply((float*)t2, (float*)P, 8, 8, 8, (float*)t3); //this step uses P as input, so I cannot put P as output also
    Matrix.Copy((float*)t3, 8, 8, (float*)P);
    
    //***  X_E = [ zeros(5,1); BiasHat] + G * Y_E; ***
    // t1 <- G*Y_E              8x1
    // t2 <- {0,0,0,0,0,Bias}   8x1
    // X_E <- t2+t1             8x1
    Matrix.Multiply((float*)G, (float*)Y_E, 8, 3, 1, (float*)t1);
    Matrix.Scale((float*)t2, 8, 8, 0);
    t2[5][0]=Bias[0]; t2[6][0]=Bias[1]; t2[7][0]=Bias[2];
    Matrix.Add((float*)t2, (float*)t1, 8, 1, (float*)X_E);
    
    //***  X_INS = X_INS - X_E(1:5); ***
    X_INS[0] = X_INS[0] - X_E[0];
    X_INS[1] = X_INS[1] - X_E[1];
    X_INS[2] = X_INS[2] - X_E[2];
    X_INS[3] = X_INS[3] - X_E[3];
    X_INS[4] = X_INS[4] - X_E[4];
  
    //***  Bias = Xhat_E(6:8); ***
    Bias[0] = X_E[5];
    Bias[1] = X_E[6];
    Bias[2] = X_E[7];
//    digitalWrite(debugPin2, LOW); //Debug on osciliscope
  }
}

void EKF_CheckConvergence()
{
  byte i,j, numParametersOK=0;
  for (i=0;i<8;i++)
  {
    if((P[i][i]<(P0[i][i]/beta)*0.3) && (P[i][i]>0)) numParametersOK++;
  }
//  Serial.println(numParametersOK);
  if(numParametersOK==8)
    EKF_convergence_OK=true;
  else
    EKF_convergence_OK=false;
}
void print_P_diag()
{
  byte i_p;
  for (i_p=2;i_p<5;i_p++) 
  {
    Serial.print(P[i_p][i_p],3);
    Serial.print("\t");
  }
  Serial.println();
}

