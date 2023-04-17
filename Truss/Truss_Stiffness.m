clear; close all; %initialization

% [nodes, elements, E, A, bcs, loads] = Tutorial_10_7();
[nodes, elements, E, A, bcs, loads] = Input_File_1();
n = size(nodes,1);
num_elements = size(elements,1);
num_dof = 2*n;

K = zeros(num_dof,num_dof);
Q = zeros(num_dof,1);
D = zeros(num_dof,1);

%known displacements
constrainedDof =[];
for i = 1:size(bcs,1)
  cdof = nodes(bcs(i,1),bcs(i,2)+2);
  constrainedDof = [constrainedDof, cdof];
  D(cdof) = bcs(i,3);
end 
%fprintf('Constrained dof are %f ', constrainedDof);

%unknown displacements
free_dof = [];
for i = 1:2*n
  if i<min(constrainedDof)
    free_dof =[free_dof, i];
  end
end

%fprintf('%f',free_dof);

%loads
for i = 1:size(loads,1)
  load_pos = nodes(loads(i,1),loads(i,2)+2);
  Q(load_pos) = loads(i,3);
end

%Global Stiffness Matrix
check = [];
dof_concerned = zeros(1,4);
for i = 1:num_elements
  near_end = elements(i,1);
  near_one = nodes(near_end,3);
  near_two = nodes(near_end,4);
  far_end = elements(i,2);
  far_one = nodes(far_end,3);
  far_two = nodes(far_end,4);
  coord = [nodes(near_end,:);nodes(far_end,:)];
  k = Elemental_truss_stiffness_1(coord,E(i),A(i));
  dof_concerned = [near_one near_two far_one far_two]; 
  check = [check;k];
  for j = 1:4
    for l = 1:4
      K(dof_concerned(1,j),dof_concerned(1,l)) += k(j,l);
    endfor
  endfor
endfor
check
disp('The K is');
K
%solution
loads_known = Q(1:length(free_dof));
K_first = K([1:length(free_dof)],[1:length(free_dof)]);
unknown_dof = zeros(length(free_dof),1);
unknown_dof = inv(K_first)*(loads_known-(K([1:length(free_dof)],[length(free_dof)+1:end])*D([length(free_dof)+1:end])));

D(1:length(free_dof),1) = unknown_dof;

Q(length(free_dof)+1:end) = K(length(free_dof)+1:end,:)*D;
%fprintf('%f ',Q);
fprintf('\n');
