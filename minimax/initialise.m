%Initialise system and controller data
function [data, controller] = initialise()
    %Get system 
    data = doubleintergrator();
    
    %Desired level of suboptimality
    data.subopt = 1;
    %Barycentric coefficients
    data.lam = sdpvar(2^(data.dim),1);
    %Initial hypercube for futher partition
    data.box.lb=-ones(data.dim, 1);
    data.box.ub=ones(data.dim, 1);
    
    % Approximation structure with 5 synchronized trees (with node-to-node interrelation):
    % - tree with stability-related information
    % - tree with hypercube's tight lower and upper bounds on vertices
    % - tree with optimal control inputs on vertices
    % - tree with optimal costs on vertices
    % - tree with decision on which axis (or edge) is sliced 
    controller = struct('stab_tree',tree(0),...
                    'part_tree',tree(data.box),...
                    'ctrl_tree',tree([]),...
                    'cost_tree',tree([]),...
                    'axis_tree',tree(0));
end


%Initialise system and controller data
function data = doubleintergrator()
    % Linear dynamics x_next = A x + B u
    A = [1 1; 0 1];
    B = [1; 0.5];
    sys = LTISystem('A', A, 'B', B);
    % State constraints
    sys.x.min = [-5; -5];
    sys.x.max = [5; 5];
    % Inputs constraint
    sys.u.min = -0.5;
    sys.u.max = 0.5;
    % Quadratic stage cost
    sys.x.penalty = QuadFunction(diag([1, 1]));
    sys.u.penalty = QuadFunction(0.01);
    sys.x.with('terminalPenalty');
    sys.x.terminalPenalty = sys.LQRPenalty;
    sys.x.with('terminalSet');
    sys.x.terminalSet=sys.LQRSet;  
    
    % MPC controller with prediction horizon = 20
    horizon=20;
    mpc = MPCController(sys, horizon);
    
    % export to YALMIP 
    data = mpc.toYALMIP;
    data.mpc = mpc;
    data.sys  = sys;
    data.dim = length(A);
end