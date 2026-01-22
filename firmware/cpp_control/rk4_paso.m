function X_next = rk4_paso(X, U, phys, h)

%Esta funcion me sirve para aplicar el metodo RK4 y ver como reponde el vector X
%Pedimos el vector X actual de estados, la entrada fisica actual de U, la
%estructura phys (m, g, I) y el paso de integracion. 

    k1 = dron_derivadas_fisicas(X, U, phys); %Calculo primera pendiente 
    k2 = dron_derivadas_fisicas(X + 0.5*h*k1, U, phys); %Calculo pendiente de punto medio extrapolando con K1
    k3 = dron_derivadas_fisicas(X + 0.5*h*k2, U, phys); %Calculo pendiente en punto medio con K2
    k4 = dron_derivadas_fisicas(X + h*k3,     U, phys); %Calculo pendiente al final del intervalo
    X_next = X + (h/6)*(k1 + 2*k2 + 2*k3 + k4);  %Promedio ponderado RK4

end
