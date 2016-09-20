function vertex = find_vertex(model_sub)

k_1 = 1;

for k = 1 : size(model_sub,1)-1
    a_1 = tan((model_sub(k+1,1) - model_sub(k,1))/(model_sub(k+1,2) - model_sub(k,2)))
    if(k+2 > size(model_sub,1))
        a_2 = tan((model_sub(1,1) - model_sub(k+1,1))/(model_sub(1,2) - model_sub(k+1,2)))
    else
        a_2 = tan((model_sub(k+2,1) - model_sub(k+1,1))/(model_sub(k+2,2) - model_sub(k+1,2)))
    end
    
    if(abs(a_1 - a_2) > pi/4)
        vertex(k_1,1) = model_sub(k+1,1);
        vertex(k_1,2) = model_sub(k+1,2);
        k_1 = k_1+1;
    end
end