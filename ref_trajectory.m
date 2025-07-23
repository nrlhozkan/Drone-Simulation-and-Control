function [pd, vd] = ref_trajectory(t, refType)
% Generates position and velocity references
% refType = 'hoverZ' | 'hoverXY' | 'circle' | ...

switch refType
    case 'hoverZ'      % 1‑m altitude step at t = 1 s
        if t < 10
            pd = [0; 0; 0];         % [x; y; z] (m)
            vd = [0; 0; 0];
        else
            pd = [0; 0; 1];
            vd = [0; 0; 0];
        end

    case 'hoverX'     % 1‑m step in X at t = 1 s
        if t < 1
            pd = [0; 0; 1];
            vd = [0; 0; 0];
        else
            pd = [1; 0; 1];
            vd = [0; 0; 0];
        end
    case 'hoverY'     % 1‑m step in Y at t = 1 s
        if t < 1
            pd = [0; 0; 1];
            vd = [0; 0; 0];
        else
            pd = [0; 1; 1];
            vd = [0; 0; 0];
        end
      
    case 'hoverXY'     % 1‑m step in X and Y at t = 1 s
        if t < 1
            pd = [0; 0; 1];
            vd = [0; 0; 0];
        else
            pd = [1; 1; 1];
            vd = [0; 0; 0];
        end
            
    case 'spiral'      % your original helix
        pd = [2*sin(0.5*t);  2*cos(0.5*t); 0.1*t];
        vd = [1*cos(0.5*t); -1*sin(0.5*t); 0.1];

    otherwise
        error('Unknown refType');
end
