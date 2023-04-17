function [nodes, elements, E, A, I, bcs, loads] = input_file_1()
  nodes= [0 4 6; 2 3 5; 4 1 2]; %[x-distance    moment-dof    y-dof] 
                                  ... associated with the node
  elements = [1 2; 2 3];
  E(1:size(elements)) = 1;
  A(1:size(elements)) = 1;
  I(1:size(elements)) = 1;

  bcs = [1 2 0; 2 2 0]; %[node   y(2)/moment(1)    magnitude]
  loads = [3 2 -5]; %[node    y(2)/moment(1)   magnitude]
endfunction
