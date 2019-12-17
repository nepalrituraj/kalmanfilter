//
//  kalman_2D.cpp
//  kalman
//
//  Created by Rituraj Nepal on 17.12.2019.
//  Copyright © 2019 welete. All rights reserved.
//

#include "kalman_2D.hpp"

// Constructor
Kalman_2D::Kalman_2D(){
    A = initA(1.f);
    B = initB(1.f);
    currentState = initCurrentState(4000, 280);
    processCov = initProcessCov(20, 5);
    kalmanGain = initKalmanGain();
    acc = 2.0 ;
}

// Deconstructor
Kalman_2D::~Kalman_2D(){
    
}


matrix<double> Kalman_2D::I(){
    matrix<double> i (2,2,0);
    i(0,0) = 1 ;
    i(1,1) = 1 ;
    return i ;
}
matrix<double> Kalman_2D::initA(double dt){
    matrix<double> a(2,2,0);
    a(0,0) = 1 ;
    a(0,1) = dt ;
    a(1,0) = 0 ;
    a(1,1) = 1 ;
    return a;
}

matrix<double> Kalman_2D::initB(double dt){
    matrix<double> b(2,1,0);
    b(0,0) = dt * dt / 2.f ;
    b(1,0) = dt ;
    return b;
}

matrix<double> Kalman_2D::initCurrentState(double position, double velocity){
    matrix<double> currentstate(2,1);
    currentstate(0,0) = position;
    currentstate(1,0) = velocity;
    return currentstate;
}

matrix<double> Kalman_2D::getCurrentState(){
    return this->currentState;
}


matrix<double> Kalman_2D::initProcessCov(double varPosition, double varVelocity){
    matrix<double> processcov(2,2,0);
    processcov(0,0) = varPosition * varPosition ; // Variance in Position
    processcov(1,1) = varVelocity * varVelocity ; // Variance in Velocity
    return processcov;
}


matrix<double> Kalman_2D::initKalmanGain(){
    matrix<double> kalmangain (2,2,0);
    return kalmangain;
}


matrix<double> Kalman_2D::updateProcessState(matrix<double> A, matrix<double>B, matrix<double> state , double control ){
    matrix<double> firstTerm(2,1,0);
    matrix<double> secondTerm(2,1,0);
    // processState = AX + Bu + ProcessError , lets assume process error to be zero
    firstTerm = prod(A,state);
    secondTerm += B;
    secondTerm *= control ;
    firstTerm += secondTerm; // Add first term with second term
    return firstTerm; // return Ax + Bu
}

// Update process covariance = A*processcov*A.transpose
matrix<double> Kalman_2D::calculateProcessCov(matrix<double> A , matrix<double> cov ){
    matrix<double> updatecov(2,2,0);
    updatecov = prod(A,cov);
    updatecov = prod(updatecov,trans(A));
    // Readjust covposition * covVelocity to Zero because it has no effect in further analysis
    updatecov(0,1) = 0.f;
    updatecov(1,0) = 0.f;
    return updatecov;
}

matrix<double> Kalman_2D::calculateKalmanGain(matrix<double> processCov, matrix<double> measurementCov){
    matrix<double> kg(2,2,0);
    matrix<double> denom(2,2,0);
    
    denom += processCov;
    denom += measurementCov;
    kg += processCov ;

    for (unsigned i = 0; i < kg.size1 (); ++ i)
        for (unsigned j = 0; j < kg.size2 (); ++ j)
            if (i == j)
                kg(i,j) =kg(i,j) / denom(i,j);

    return kg;
}


void Kalman_2D::predictState(matrix<double> measuredState,matrix<double> measuredCov){
    this->currentState = updateProcessState(this->A, this->B, this->currentState, this->acc); // Process State = Ax + Bu
    this->processCov = calculateProcessCov(this->A,this->processCov);
    this->kalmanGain = calculateKalmanGain(this->processCov, measuredCov);
   
    
    
    /*    Now We predict next State using Kalman Gain and assign this to currernt State for Next Iteration */
    matrix<double> diff (2,1,0);
    /*   difference = measuredState - currentState */
    diff = measuredState ;
    diff -= this->currentState;
    
    diff = prod(this->kalmanGain,diff); // Multiply with kalman gain to update values
    // Add previousState to update currentState
    this->currentState += diff ; // This step now update the currentState using kalman gain and previous state
    /* We need to update Covariance Matrix for next Iteration */
    this->processCov = updateProcessCov(this->kalmanGain);
    
    /* We updated current state, kalman gain, and covariance matrix */
}

void Kalman_2D::printState(){
    cout << this->currentState << endl;
}

// Update process covariance with kalman gain
matrix<double> Kalman_2D::updateProcessCov(matrix<double> kalmanGain){
    matrix<double> processcov(2,2,0);
    matrix<double> i(2,2,0);
    i = I();
    i -= kalmanGain;
    processcov = prod(i,this->processCov);
    return processcov ;
}
