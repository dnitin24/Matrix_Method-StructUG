% TRUSS ANALYSIS
clear global;clc;

%.....INPUT....
COORD=[0 0; 10 0; 20 0; 0 12; 10 12; 20 12];
CON=[1 2; 2 3; 4 5; 5 6; 1 4; 2 4; 1 5; 2 5;3 5;2 6;3 6];
EQ=[10 11; 1 2; 3 12; 4 5; 6 7; 8 9];
NR=3;
NE=size(CON,1);
NN=size(COORD,1);
EA=[2 3 3 3 2 2 3 2 3 4 2]'*10^3;

Pf=[0 0 0 0 0 0 -10 0 0]';
Ur=[0 0 0]';
scale = 20;
%........CALCULATION........

NOS=NE+NR-2*NN;
NOK=2*NN-NR;

%LENGTH OF ELEMENT, ID ARRAY AND Stiffness Matrix

ID=zeros(NE,4);
L=zeros(NE,1);
NDOF=2*NN;
K=zeros(NDOF,NDOF);
for k=1:NE
    i=CON(k,1);
    j=CON(k,2);
    dx=COORD(j,1)-COORD(i,1);
    dy=COORD(j,2)-COORD(i,2);
    L(k)=sqrt(dx^2+dy^2);
    ID(k,1:2)=EQ(i,1:2);
    ID(k,3:4)=EQ(j,1:2);
    a=[-dx/L(k) -dy/L(k) dx/L(k) dy/L(k)];
    ES=a'.*EA(k)/L(k)*a;
    for m=1:4
        for n=1:4
            mi=ID(k,m);
            ni=ID(k,n);
            K(mi,ni)=K(mi,ni)+ES(m,n);
        end
    end
end

Kff(1:NOK,1:NOK)=K(1:NOK,1:NOK);
Kfr(1:NOK,1:NDOF-NOK)=K(1:NOK,NOK+1:NDOF);
Krf=Kfr';
Krr(1:NDOF-NOK,1:NDOF-NOK)=K(NOK+1:NDOF,NOK+1:NDOF);

%DEFORMATION, INTERNAL FORCES AND SUPPORT REACTIONS
Uf=Kff\Pf;
U=[Uf; Ur];
N=zeros(NE,1);
for k=1:NE
    i=CON(k,1);
    j=CON(k,2);
    dx=COORD(j,1)-COORD(i,1);
    dy=COORD(j,2)-COORD(i,2);
    a=[-dx/L(k) -dy/L(k) dx/L(k) dy/L(k)];
    u=zeros(4,1);
    for m=1:4
        u(m)=U(ID(k,m));
    end
    N(k)=EA(k)/L(k).*a*u;
end
R=Krf*Uf+Krr*Ur;

%--Plot Structure ----
f1 = figure();
%f1.WindowState = 'maximized';
NCOORD = zeros(size(COORD));
UCOORD = zeros(size(COORD));
for n = 1:NN
    NCOORD(n,1) = COORD(n,1) + scale*U(EQ(n,1));
    NCOORD(n,2) = COORD(n,2) + scale*U(EQ(n,2));
    UCOORD(n,1) = U(EQ(n,1));
    UCOORD(n,2) = U(EQ(n,2));
    marker = plot(NCOORD(n,1), NCOORD(n,2), '.',...
        'MarkerSize', scale, 'MarkerEdgeColor', 'r')
    uistack(marker,'top');
    text(NCOORD(n,1)+0.5, NCOORD(n,2)-0.75, ...
        {['Ux=' num2str(round(12*UCOORD(n,1),2))];...
        ['Uy=' num2str(round(12*UCOORD(n,2),2))]});
    hold on
end

for k = 1:NE
    i=CON(k,1);
    j=CON(k,2);
    x = [COORD(i,1) COORD(j,1)];
    y = [COORD(i,2) COORD(j,2)];
    plot(x, y, 'Color', uint8([134 136 138]), 'LineWidth', 2);
    % hold on
    ux = [NCOORD(i,1) NCOORD(j,1)];
    uy = [NCOORD(i,2) NCOORD(j,2)];
    plot(ux, uy, 'r--', 'LineWidth', 1.5);
    % unistack(undeformed, 'down', k+NE*2)
    % unistack(deformed, 'down', k+NE)
end

xlim([0-5, max(COORD(:,1))+5]);
ylim([0-5, max(COORD(:,2))+5]);
axis equal;
    
%----RESULT-----
%INTERNAL FORCES
fprintf('          Internal Force     \n');
fprintf('.................................\n');
fprintf('| ELEMENT | AXIAL FORCE | UNIT | \n');
fprintf('.................................\n');
for i = 1:NE
    if i<10
        if N(i) <0
            fprintf('|  %d.     |  %0.3f     | kips | \n',i,N(i));
        else
            fprintf('|  %d.     |   %0.3f     | kips | \n',i,N(i));
        end
    else
        if N(i) <0
            fprintf('| %d.     |  %0.3f     | kips | \n',i,N(i));
        else
            fprintf('| %d.     |   %0.3f     | kips | \n',i,N(i));
        end
        
    end 
    
end

fprintf('NOTE:\n');
fprintf('Compression : Negative\n');
fprintf('Tension     : Positive\n');
fprintf('................................\n');

%SUPPORT REACTIONS
fprintf('         SUPPORT REACTION     \n');
fprintf('.................................\n');
fprintf('|   DOF   |    FORCE    | UNIT | \n');
fprintf('.................................\n');
for i = 1:size(Ur,1)
    if NDOF - (size(Ur,1)-i)<10
        if R(i) <0
            fprintf('|  %d.     |  %0.3f     | kips | \n',NDOF - (size(Ur,1)-i),R(i));
        else
            fprintf('|  %d.     |   %0.3f     | kips | \n',NDOF - (size(Ur,1)-i),R(i));
        end
    else
        if R(i) <0
            fprintf('| %d.     |  %0.3f     | kips | \n',NDOF - (size(Ur,1)-i),R(i));
        else
            fprintf('| %d.     |   %0.3f     | kips | \n',NDOF - (size(Ur,1)-i),R(i));
        end
        
    end 
    
end

fprintf('NOTE:\n');
fprintf('Negative : Downward\n');
fprintf('Positive : Upward\n');
fprintf('................................\n')




%DEFORMATION
fprintf('            DISPLACEMENT       \n');
fprintf('.................................\n');
fprintf('|   DOF   | DISPLACEMENT| UNIT | \n');
fprintf('.................................\n');
for i = 1:NDOF
    if i<10
        if U(i) <0
            fprintf('|  %d.     |  %0.3f     |  in. | \n',i,12*U(i));
        else
            fprintf('|  %d.     |   %0.3f     |  in. | \n',i,12*U(i));
        end
    else
        if U(i) <0
            fprintf('| %d.     |  %0.3f     |  in. | \n',i,12*U(i));
        else
            fprintf('| %d.     |   %0.3f     |  in. | \n',i,12*U(i));
        end
        
    end 
    
end

fprintf('NOTE:\n');
fprintf('X Direction : Left to Right\n');
fprintf('Y Direction : Top to Bottom\n');
fprintf('................................\n')




