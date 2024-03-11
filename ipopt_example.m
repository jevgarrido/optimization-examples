function ipopt_example()

% IPOPT solver
% https://github.com/coin-or/Ipopt
%
% See ipopt interface here:
% https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Research_Onder/Downloads/IPOPT/IPOPT_MatlabInterface_V0p1.pdf
% 
% Detailed list of IPOPT options
% https://coin-or.github.io/Ipopt/OPTIONS.html
%
% MATLAB toolbox with precompiled ipopt mex
% https://www.mathworks.com/matlabcentral/fileexchange/53040-ebertolazzi-mexipopt
%
% Linear programming examples (2 of them)
% https://math.mit.edu/~goemans/18310S15/lpnotes310.pdf
% 
% Convex Optimization book
% https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf


%% Problem Formulation
auxdata      	= struct();

initial_guess	= [ 0; 0 ];

options.lb    = [ 0; 0 ];           % xl
options.ub    = [ Inf; Inf ];       % xu

options.cl    = [ 24; 15; 3 ];  	% gl: [ 24, 15, 3 ] is more interesting than [ 8, 15, 3 ]
options.cu    = [ Inf; Inf; Inf ];	% gu

auxdata.num_variables	= length( initial_guess );
auxdata.num_constraints = length( options.cl );

auxdata.cost_gradient	= [ 0.6, 0.35 ];
auxdata.Jacobian        = [ 5, 7; 4, 2; 2, 1 ];


%% Mathematical functions
funcs.objective       	= @(x) f( x, auxdata );
funcs.gradient          = @(x) grad_f( x, auxdata );
funcs.constraints       = @(x) h( x, auxdata );
funcs.jacobian          = @(x) grad_h( x, auxdata );
funcs.jacobianstructure	= @( ) grad_h_sparsity( auxdata );


%% Solver Options
options.ipopt.max_iter          = 1000;
options.ipopt.tol               = 1e-9;
options.ipopt.acceptable_tol	= 1e-9;

% Necessary to avoid having to implement Hessian by hand.
options.ipopt.hessian_approximation = 'limited-memory';


%% Solver Call
[ linex, info ]     = ipopt( initial_guess, funcs, options );


%% Results/Plotting
[ surfx, surfy ]	= meshgrid( linspace(0,5,50), linspace(0,5,50));

v	= [ 7, -5; 2, -4; 1, -2 ];
p0	= [ 0, 24/7; 0, 15/2; 0, 3 ];
p1	= p0 + 5*v;

close all

figure(1)
contour( surfx, surfy, 0.6*surfx + 0.35*surfy )
hold on

plot( [p0(1,1), p1(1,1)], [p0(1,2), p1(1,2)], "linewidth", 2 )
plot( [p0(2,1), p1(2,1)], [p0(2,2), p1(2,2)], "linewidth", 2 )
plot( [p0(3,1), p1(3,1)], [p0(3,2), p1(3,2)], "linewidth", 2 )

plot( linex(1), linex(2), "*k", "linewidth", 2 )

hold off
axis equal
ylim([0 5]);
xlim([0 5]);
xlabel("x1")
ylabel("x2")

figure(2)
bar( info.lambda )

end

function cost = f( x, auxdata )
    cost = auxdata.cost_gradient * x;
end

function cost_gradient = grad_f( ~, auxdata )
    cost_gradient = auxdata.cost_gradient;
end

function constraints = h( x, auxdata )
    constraints = auxdata.Jacobian * x;
end

function jacobian = grad_h( ~ , auxdata )
    jacobian = sparse( auxdata.Jacobian );
end

function jacobian_sparsity = grad_h_sparsity( auxdata )
    jacobian_sparsity = sparse( ones( auxdata.num_constraints, auxdata.num_variables ) );
end

