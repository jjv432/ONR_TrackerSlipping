%% Function for finding dx/dt

function neg_vx_indices = find_slip(data)

   vx = data.vx;
   neg_vx_indices = find(vx<0);

end