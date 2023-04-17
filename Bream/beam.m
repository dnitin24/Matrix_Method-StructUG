clc;  clear all; %initialization

[nodes, elements, E, A, I, bcs, loads] = Tutorial_10_9();

n = size(nodes,1);
num_ele = size(elements,1);
ndof = 2*n;

K = zeros(ndof);
Q = zeros(ndof,1);
U = zeros(ndof,1);

%constarined dofs
constrained_dof = [];
for i = 1:size(bcs,1)
  cdof = nodes(bcs(i,1), bcs(i,2)+1);
  constrained_dof = [constrained_dof,cdof];
  U(cdof) = bcs(i,3);
endfor

%free dofs
free_dof = [];
for i = 1:2*n
  if i<min(constrained_dof)
   free_dof = [free_dof,i]; 
  end
endfor 

%update loads vector
for i = 1:size(loads,1)
  load_dof = nodes(loads(i,1), loads(i,2)+1);
  Q(load_dof) = loads(i,3);
endfor

%Global Stiffness Matrix
concernedDof = [];
check = [];
for i = 1:num_ele
  near_end = elements(i,1);
  near_one = nodes(near_end,2);
  near_two = nodes(near_end,3);
  far_end = elements(i,2);
  far_one = nodes(far_end,2);
  far_two = nodes(far_end,3);
  coord = [nodes(near_end); nodes(far_end)];
  k = Elemental_Beam_Stiffness(coord,E(i),I(i));
  concernedDof = [near_two near_one far_two far_one]
  check = [check;k];
  for j =1:4
    for l=1:4
      K(concernedDof(1,j),concernedDof(1,l)) += k(j,l);
    endfor
  endfor
endfor
check
K
%solution
loads_known = Q(1:length(free_dof));
K_first = K([1:length(free_dof)],[1:length(free_dof)]);
K_second = K([1:length(free_dof)],[length(free_dof)+1:end]);
unknown_dof = inv(K_first)*(loads_known-(K_second*U([length(free_dof)+1:end])));

U(1:length(free_dof)) = unknown_dof;
Q(length(free_dof)+1:end) = K(length(free_dof)+1:end,:)*U;
