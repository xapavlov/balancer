function [stability, U, J, dJdx] = certify(data, box)

    % Compute vertices of the hyppercube
    dim=length(box.lb);
    V=box.lb*ones(1, 2^dim);
    combinations=(dec2bin(0:(2^dim-1)) - '0')';
    edges=(box.ub-box.lb)*ones(1, 2^dim);
    V=V+combinations.*edges;
    
    %Compute optimal costs and optimal controls
    J = nan*ones(1,2^data.dim);
    for i = 1:2^data.dim
      con=data.constraints + [(data.variables.x(:,1)==V(:,i)):'IC'];
      mplp_solution = optimize(con, data.objective, sdpsettings('verbose',0));
      if mplp_solution.problem
        warning('Computational problem');
      else
        u=value(data.variables.u);
        U(:,i)=u(:,1); %keep only the first input of the sequence
        J(i) = value(data.objective);
        dJdx(:,i)=-dual(con('IC'));
      end
    end
    
    %Check if box is inside the terminal set
    termSet = data.sys.x.terminalSet;
    if all(termSet.contains(V))  
      stability=1; %LQR stable 
      return
    end
    
    %Check if the box is outside feasible set
    if all(isnan(J)) 
      control=[];
      J=[];
      dJdx=[];
      stability=-2; %outside feasible set
      return
    end
    
    if any(isnan(J)) %Terminating on minimum box size
      stability=0; %unstable
    else
      x = data.variables.x(:,1);
      lam=data.lam;
      Q = data.mpc.model.x.penalty.H;
      R = data.mpc.model.u.penalty.H;
      obj = data.objective + data.subopt*x'*Q*x - J*lam;
      con = data.constraints + [lam >= 0, sum(lam) == 1];
      con = con + [x == V*lam];

      certificate = optimize(con, obj, sdpsettings('verbose',0));
      
      if certificate.problem
        warning('Computational problem');
        stab_margin= -Inf;
      else
        stab_margin = value(obj);   %compute stability margin
      end
      
      %give worstcase certificase if positive, otherwise try minimax certificate
      if stab_margin>0 
        stability=2; %worstcase stable
      else 
        %formulate and solve multiparametric LP problem
        mplp = Opt([data.lam>=0, sum(data.lam)==1, V*data.lam==x, box.lb<=x<=box.ub], J*lam, x, data.lam);
        mplp_solution = mplp.solve();
        
        %certify critical regions
        crit_regions=1:numel(mplp_solution.xopt.Set);
        for i=crit_regions
          %Get lambda as a function of x
          lam_func=mplp_solution.xopt.Set(i).getFunction('primal');
          lam=lam_func.F*x+lam_func.g;
          %Get set polyhedra representation of the critical region as
          %A_poly*x<=b_poly
          A_poly=mplp_solution.xopt.Set(i).A;
          b_poly=mplp_solution.xopt.Set(i).b;       
          
          stagecosts=diag(V'*Q*V)'+diag(U'*R*U)';
          upperbound=(J-stagecosts)*lam;

          obj = data.objective + (data.subopt-1)*(x'*Q*x + (U*lam)'*R*(U*lam)) - upperbound;
          con = data.constraints + [A_poly*x<= b_poly];
          certificate = optimize(con, obj, sdpsettings('verbose',0));
          
          if (certificate.problem)
            switch (certificate.problem)
                case 4
                    disp('Numerical problems, keep results anyway')
                    minimax_margin=value(obj);
                otherwise
                    warning('Infeasible or nonconvex problem');
                    minimax_margin=-Inf;
            end
          else
              minimax_margin=value(obj);
          end
          
          %Stop procedure if the critical region has negative stability margin
          if minimax_margin<0 
            stability=0; %unstable
            break;
          end
          
          %Give minimax certificate if reached the last critical region
          %with all positive minimax stability margins
          if i==numel(mplp_solution.xopt.Set) 
            stability=3;
          end 
        end
        
      end
    end
end
