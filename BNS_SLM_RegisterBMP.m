function [ phase_out ] = BNS_SLM_RegisterBMP(phase_in)
    [m n] = size(phase_in);
    t_phase_in = padarray( phase_in, [round(m/2) round(n/2)] );
    t_phase_in = circshift( t_phase_in, [-1, -1] );
    phase_out = t_phase_in( (m-m/2+1):(m+m/2), (n-n/2+1):(n+n/2) );
end