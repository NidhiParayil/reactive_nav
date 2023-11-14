from scripts import pybullet_env 
from scipy import integrate
from scipy.optimize import fsolve, minimize
import numpy as np


class OptimalControl():
    def __init__(self, A, B, Q, R):
        # self.weighted_G = G + R 
        self.A = A 
        self.B = B
        self.R =R
        self.I = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
      
    

    def get_optimal_control(self, t,x0,x1):
        # T =3
        # print(T)
        T = t[0]
        # x0 = np.asmatrix([0,1,0,1]).T
        a_a =np.matmul(self.A,self.A)
        a_a_t=np.matmul(self.A.T,self.A.T)
        term1 = self.I + self.A * T + a_a*(T)**2/2+ np.matmul(a_a,self.A)*(T)**3/6
        # x1= np.asmatrix([2,2,0,1]).T
        self.I_x = np.asmatrix([1,0,0,0]).T
        self.cnt = np.asmatrix([1,1,1,1]).T

        def G_ode(t_, T):
            t__ = T- t_
            a_a =np.matmul(self.A,self.A)
            a_a_t=np.matmul(self.A.T,self.A.T)
            term1 = self.I + self.A * t__ + a_a*(t__)**2/2+ np.matmul(a_a,self.A)*(t__)**3/6
            term2 = self.I + self.A.T * t__ + a_a_t*(t__)**2/2+ np.matmul(a_a_t,self.A.T)*(t__)**3/6
            dG_dt_ = np.matmul(np.matmul(np.matmul(np.matmul(term1,self.B),self.R), self.B.T), term2)
            return dG_dt_

        def get_x_bar(t_, T):
            t__ = T- t_
            a_a =np.matmul(self.A,self.A)
            a_a_t=np.matmul(self.A.T,self.A.T)
            term_xintegral = np.matmul(self.I+ self.A * t__ + a_a*(t__)**2/2+ np.matmul(a_a,self.A)*(t__)**3/6,self.cnt) 
            return term_xintegral


        

            
        self.G, _ = integrate.quad_vec(G_ode, 0, T, args=(T)) 
        xbar_term, _= integrate.quad_vec(get_x_bar, 0, T, args=(T)) 
        
        self.x_bar = np.matmul(term1, x0)+xbar_term
        exp_term =  self.I_x + self.A * T + a_a*(T)**2/2+ np.matmul(a_a,self.A)*(T)**3/6
        # self.u =  np.matmul(self.R, np.matmul(self.B.T, np.matmul(exp_term,(np.matmul(self.G,(x1-self.x_bar))))))
        # def get_c(t,T,x1):
    
        d_tau = np.matmul(np.linalg.inv(self.G), (self.x_bar))
            # print(d_tau, self.G, (x1-self.x_bar))
        c = 1- 2*np.matmul((np.matmul(self.A,x1)).T, d_tau) - np.matmul(d_tau.T,np.matmul(self.B,np.matmul(self.R,np.matmul(self.B.T,d_tau))))
        # sol = minimize(self.get_optimal_control, T_guess,method='BFGS')
        return c
        # print("tau is",self.c)
 

    def get_optimal_time(self,x0, x1):
        # c = self.get_optimal_control(self, T)
        T_guess = 5
        sol = minimize(self.get_optimal_control, T_guess,method='BFGS', args=(x0, x1))
        optimal_time = sol.x
        optimal_cost = self.get_optimal_control(sol.x,x0, x1)
        # print(optimal_cos)
        return optimal_time, optimal_cost

    def get_optimal_traj(self,x0, x1):
        optimal_time, optimal_cost = self.get_optimal_time(x0, x1)
        d_tau = np.matmul(np.linalg.inv(self.G), (self.x_bar))
