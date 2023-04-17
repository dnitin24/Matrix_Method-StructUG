function k = Truss_single_stiffness_1(coord, E, A)
  x1 = coord(1,1);
  x2 = coord(2,1);
  y1 = coord(1,2);
  y2 = coord(2,2);
  len = sqrt((x1-x2)**2 + (y1-y2)**2);
  lambda_x = (x2-x1)/len;
  lambda_y = (y2-y1)/len;
  T = [lambda_x lambda_y 0 0; 0 0 lambda_x lambda_y];
  k_dash = [1 -1; -1 1];
  k = (A*E/len)*(T'*k_dash*T);
endfunction
