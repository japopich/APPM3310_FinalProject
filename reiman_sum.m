
for i = 2:length(A)
    dx = i - (i-1); 
    h = A(i); 
    V_r(i) = h*dx; 
end 

for i = 2:length(V)
    dx = i - (i-1); 
    h = V_r(i); 
    X_r(i) = h*dx; 
end