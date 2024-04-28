import numpy as np

def build_Y_phi_ARX(u,y,ord):
    aux2_y = np.zeros((y.size-ord,1))
    aux2_u = np.zeros((y.size-ord,1))
    for o in range(ord):
        aux_y  = -y[ord-1-o:-1-o]
        aux_u  = u[ord-1-o:-1-o]
        aux2_y = np.concatenate((aux2_y,aux_y),axis=1)
        aux2_u = np.concatenate((aux2_u,aux_u),axis=1)
    aux2_y = aux2_y[:,1:]
    aux2_u = aux2_u[:,1:]
    phi = np.concatenate((aux2_y,aux2_u), axis=1)
    return y[ord:].copy(), phi

def build_Y_phi_FIR(u,y,ord):
    aux2_u = np.zeros((u.size-ord,1))
    for o in range(ord):
        aux_u  = u[ord-1-o:-1-o]
        aux2_u = np.concatenate((aux2_u,aux_u),axis=1)
    phi  = aux2_u[:,1:]
    return y[ord:].copy(), phi

def calcular_theta_min_quad(y,p):
    pT_p = np.matmul(p.T,p)
    pT_p_inv = np.linalg.inv(pT_p)
    pT_y = np.matmul(p.T,y)
    t_h = np.matmul(pT_p_inv,pT_y)
    return t_h

def estimate_output(y,u,theta,ord):
    y_hat = np.zeros(y.shape)
    for i in range(y.size-ord):
        for o in range(ord):
            y_hat[i+ord] += -theta[o]*y_hat[i+ord-1-o] + theta[o+ord]*u[i+ord-1-o]        
    return y_hat

def estimate_output_FIR(u,theta,ord):
    y_hat = np.zeros(u.shape)
    for i in range(u.size-ord):
        for o in range(ord):
            y_hat[i+ord] +=  theta[o]*u[i+ord-1-o]        
    return y_hat

def estimate_output_ed(y,u,theta):
    y_hat_ed = np.zeros(y.shape)
    a1, a2, a3, a4, b0 = theta
    for i in range(y_hat_ed.size-2):
        y_hat_ed[i+2] += -a1*y_hat_ed[i+1] - a2*y_hat_ed[i] - a3*y_hat_ed[i]*y_hat_ed[i+1] - a4*y_hat_ed[i]**2 + b0*u[i]
    return y_hat_ed

def cost_func(y,y_hat):
    return np.sum(np.power(y - y_hat, 2))/y.size

def ARX_order_cost(u,y,ord):
    Y, phi = build_Y_phi_ARX(u,y,ord)
    theta_hat = calcular_theta_min_quad(Y,phi)
    y_hat = estimate_output(y,u, theta_hat, ord)
    V = cost_func(y,y_hat)
    return V

def find_best_ARX_order(u,y,ord,thr,v):
    best_order = 0
    for o in range(ord):
        V = ARX_order_cost(u,y,o+1)
        if v: print(f'ARX {o+1}ª ordem V = {V}') 
        if o > 0:
            if ((V_ant - V) < 0 or V > V_ant*(1-thr)):
                best_order = o
                break
        V_ant = V
    if not best_order:
        best_order = ord
    if v: print(f'{best_order} é a melhor ordem')
    return best_order

def FIR_order_cost(u,y,ord):
    Y, phi = build_Y_phi_FIR(u,y,ord)
    theta_hat = calcular_theta_min_quad(Y,phi)
    y_hat = estimate_output_FIR(u, theta_hat, ord)
    V = cost_func(y,y_hat)
    return V

def find_best_FIR_order(u,y,ord,thr,v):
    best_order = 0
    for o in range(ord):
        V = FIR_order_cost(u,y,o+1)
        if v: print(f'FIR {o+1}ª ordem V = {V}') 
        if o > 0:
            if ((V_ant - V) < 0 or V > V_ant*(1-thr)):
                best_order = o
                break
        V_ant = V
    if not best_order:
        best_order = ord
    if v: print(f'{best_order} é a melhor ordem')
    return best_order

def build_Y_phi_ed_load(u,y):
    Y_ed_ = y[2:]
    phi_ed = np.concatenate((-y[1:-1],-y[:-2],-y[1:-1]*y[:-2],-y[:-2]**2,u[:-2]), axis=1)
    return Y_ed_, phi_ed

def build_Y_phi_ed_empty(u,y):
    Y_ed_ = y[2:]
    phi_ed = np.concatenate((-y[1:-1],-y[:-2],u[:-2]), axis=1)
    return Y_ed_, phi_ed

def estimate_output_ed_empty(y,u,theta_ed):
    y_hat_ed = np.zeros(y.shape)
    a1, a2, b0 = theta_ed
    for i in range(y_hat_ed.size-2):
        y_hat_ed[i+2] += -a1*y_hat_ed[i+1] - a2*y_hat_ed[i] + b0*u[i]
    return y_hat_ed