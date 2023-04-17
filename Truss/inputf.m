function [nodes, elements, E, A, bcs, loads] = Input_file_1()
  nodes = [ 0 0 1 2; 3 0 3 4; 3 4 5 6]; %[x y dof1 dof2]
  elements = [ 1 2; 1 3];   
  E(1:size(elements,1))=1;
  A(1:size(elements,1))=1;
  bcs = [2 1 0; 2 2 0; 3 1 0; 3 2 0]; %boundary conditions [node dof(x(1)ory(2) dis]
  loads = [1 2 -2]; %[node dof magnitude]
endfunction
